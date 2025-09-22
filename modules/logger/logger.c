#include "logger.h"
#include <hal.h>
#include <stdarg.h>
#include <string.h>
#include <chprintf.h>

// FATFS types/functions are available via modules/uSD/uSD.h

#include <modules/uSD/uSD.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/pubsub/pubsub.h>
#include <common/helpers.h>
#ifdef MODULE_UAVCAN_DEBUG_ENABLED
#include <modules/uavcan_debug/uavcan_debug.h>
#define LOGGER_DEBUG(level, fmt, ...) uavcan_send_debug_msg(level, "LOG", fmt, ##__VA_ARGS__)
#else
#define LOGGER_DEBUG(level, fmt, ...)
#endif

#ifndef LOGGER_WORKER_THREAD
#error Please define LOGGER_WORKER_THREAD in framework_conf.h.
#endif

#define WT LOGGER_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

#ifdef LOGGER_PUBSUB_TOPIC_GROUP
PUBSUB_TOPIC_GROUP_DECLARE_EXTERN(LOGGER_PUBSUB_TOPIC_GROUP)
#endif

#ifndef LOGGER_BASE_DIR
#define LOGGER_BASE_DIR "/LOGS"
#endif

#ifndef LOGGER_ROTATE_BYTES
#define LOGGER_ROTATE_BYTES (0xFFFFFFFFu)
#endif

#ifndef LOGGER_MIN_FREE_BYTES
#define LOGGER_MIN_FREE_BYTES (8*1024*1024)
#endif

#ifndef LOGGER_MAX_PREFIX_LEN
#define LOGGER_MAX_PREFIX_LEN 8
#endif

#ifndef LOGGER_MAX_SUFFIX_LEN
#define LOGGER_MAX_SUFFIX_LEN 3
#endif

#ifndef LOGGER_FMT_SET_SIZE
#define LOGGER_FMT_SET_SIZE 16
#endif

struct logger_msg_s {
	char prefix[LOGGER_MAX_PREFIX_LEN+1];
	char suffix[LOGGER_MAX_SUFFIX_LEN+1];
	uint32_t payload_len;
	uint8_t payload[];
};

struct open_file_s {
	char prefix[LOGGER_MAX_PREFIX_LEN+1];
	char suffix[LOGGER_MAX_SUFFIX_LEN+1];
	FIL fp;
	uint32_t index;
	uint64_t bytes_written;
	uint32_t fmt_hashes[LOGGER_FMT_SET_SIZE];
	struct open_file_s* next;
};

MEMORYPOOL_DECL(logger_open_file_pool, sizeof(struct open_file_s), PORT_NATURAL_ALIGN, chCoreAllocAlignedI);
static struct open_file_s* open_file_list_head;

static struct pubsub_topic_s log_msg_topic;
static struct worker_thread_listener_task_s log_msg_listener_task;
static void log_msg_handler(size_t msg_size, const void* buf, void* ctx);

static bool logger_fmt_seen_add(struct open_file_s* f, uint32_t name_hash);

static bool logger_ensure_base_dir(void) {
	FATFS* fs = uSD_get_filesystem();
	if (!fs) {
		return false;
	}
	(void)fs;
	FRESULT res = f_mkdir(LOGGER_BASE_DIR);
	(void)res;
	return true;
}

static uint64_t logger_get_free_bytes(void) {
	DWORD free_clust = 0;
	FATFS* fs;
	if (f_getfree("/", &free_clust, &fs) != FR_OK || fs == NULL) {
		return 0;
	}
#if FF_MAX_SS != FF_MIN_SS
	uint32_t sector_size = fs->ssize;
#else
	uint32_t sector_size = 512;
#endif
	uint64_t sectors_per_cluster = (uint64_t)fs->csize;
	return (uint64_t)free_clust * sectors_per_cluster * (uint64_t)sector_size;
}

static void logger_build_path(char* dst, size_t dst_len, const char* prefix, uint32_t index, const char* suffix) {
	chsnprintf(dst, dst_len, LOGGER_BASE_DIR "/%s_%u.%s", prefix, (unsigned)index, suffix);
}

static bool logger_path_exists(const char* path) {
	FILINFO fno;
	return f_stat(path, &fno) == FR_OK;
}

static uint32_t logger_find_next_index(const char* prefix, const char* suffix) {
	char path[64];
	uint32_t idx = 1;
	while (true) {
		logger_build_path(path, sizeof(path), prefix, idx, suffix);
		if (!logger_path_exists(path)) {
			return idx;
		}
		idx++;
		if (idx == 0) {
			return 1;
		}
	}
}

static bool logger_delete_oldest_for_prefix(const char* prefix, const char* suffix) {
	DIR dir;
	FILINFO fno;
	if (f_opendir(&dir, LOGGER_BASE_DIR) != FR_OK) {
		return false;
	}
	uint32_t oldest_idx = 0;
	char oldest_name[64] = {0};
	for (;;) {
		FRESULT r = f_readdir(&dir, &fno);
		if (r != FR_OK || fno.fname[0] == 0) {
			break;
		}
		const char* name = fno.fname;
		size_t prelen = strlen(prefix);
		size_t suflen = strlen(suffix);
		if (strncmp(name, prefix, prelen) != 0) {
			continue;
		}
		if (name[prelen] != '_') {
			continue;
		}
		const char* p = name + prelen + 1;
		uint32_t idx = 0;
		bool any = false;
		while (*p >= '0' && *p <= '9') {
			any = true;
			uint32_t d = (uint32_t)(*p - '0');
			uint32_t next = idx*10u + d;
			if (next < idx) { any = false; break; }
			idx = next;
			p++;
		}
		if (!any || *p != '.') {
			continue;
		}
		if (strncmp(p+1, suffix, suflen) != 0 || (p[1+suflen] != '\0')) {
			continue;
		}
		if (oldest_idx == 0 || idx < oldest_idx) {
			oldest_idx = idx;
			chsnprintf(oldest_name, sizeof(oldest_name), LOGGER_BASE_DIR "/%s", name);
		}
	}
	f_closedir(&dir);
	if (oldest_idx == 0) {
		return false;
	}
	return f_unlink(oldest_name) == FR_OK;
}

static struct open_file_s* logger_open_or_get_file(const char* prefix, const char* suffix, size_t upcoming_write_len) {
	struct open_file_s* f = open_file_list_head;
	while (f) {
		if (strncmp(f->prefix, prefix, LOGGER_MAX_PREFIX_LEN) == 0 && strncmp(f->suffix, suffix, LOGGER_MAX_SUFFIX_LEN) == 0) {
			break;
		}
		f = f->next;
	}

	if (!f) {
		chSysLock();
		f = chPoolAllocI(&logger_open_file_pool);
		if (!f) {
			chPoolAddI(&logger_open_file_pool, chCoreAllocI(sizeof(struct open_file_s)));
			f = chPoolAllocI(&logger_open_file_pool);
		}
		chSysUnlock();
		if (!f) {
			return NULL;
		}
		memset(f, 0, sizeof(*f));
		strncpy(f->prefix, prefix, LOGGER_MAX_PREFIX_LEN);
		strncpy(f->suffix, suffix, LOGGER_MAX_SUFFIX_LEN);
		f->index = logger_find_next_index(prefix, suffix);
		char path[64];
		logger_build_path(path, sizeof(path), prefix, f->index, suffix);
		if (f_open(&f->fp, path, FA_CREATE_ALWAYS | FA_WRITE | FA_OPEN_APPEND) != FR_OK) {
			chSysLock();
			chPoolFreeI(&logger_open_file_pool, f);
			chSysUnlock();
			return NULL;
		}
		f->bytes_written = f->fp.fptr;
		memset(f->fmt_hashes, 0, sizeof(f->fmt_hashes));
		LINKED_LIST_APPEND(struct open_file_s, open_file_list_head, f);
	}

	uint64_t needed = (uint64_t)upcoming_write_len + sizeof(uint32_t) + 2;
	if (f->bytes_written + needed > (uint64_t)LOGGER_ROTATE_BYTES) {
		f_close(&f->fp);
		f->index++;
		char path[64];
		logger_build_path(path, sizeof(path), prefix, f->index, suffix);
		while (logger_get_free_bytes() < LOGGER_MIN_FREE_BYTES + needed) {
			if (!logger_delete_oldest_for_prefix(prefix, suffix)) {
				break;
			}
		}
		f_open(&f->fp, path, FA_CREATE_ALWAYS | FA_WRITE | FA_OPEN_APPEND);
		f->bytes_written = f->fp.fptr;
	}

	while (logger_get_free_bytes() < LOGGER_MIN_FREE_BYTES + needed) {
		if (!logger_delete_oldest_for_prefix(prefix, suffix)) {
			break;
		}
	}

	return f;
}

static uint16_t logger_crc16(const void* data, size_t len) {
	return crc16_ccitt(data, len, 0);
}

static void logger_file_write_record(struct open_file_s* f, const void* payload, uint32_t payload_len) {
	UINT bw;
	uint16_t crc = logger_crc16(payload, payload_len);
	f_write(&f->fp, &payload_len, sizeof(payload_len), &bw);
	f_write(&f->fp, &crc, sizeof(crc), &bw);
	f_write(&f->fp, payload, payload_len, &bw);
	f->bytes_written += sizeof(payload_len) + sizeof(crc) + payload_len;
    {
        FRESULT fr = f_sync(&f->fp);
        if (fr != FR_OK) {
            LOGGER_DEBUG(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR, "sync err %u", (unsigned)fr);
        }
    }
}

static void log_msg_handler(size_t msg_size, const void* buf, void* ctx) {
	UNUSED(ctx);
	if (msg_size < sizeof(struct logger_msg_s)) {
        LOGGER_DEBUG(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_WARNING, "msg too small %u", (unsigned)msg_size);
		return;
	}
	const struct logger_msg_s* msg = buf;
	const uint8_t* payload = msg->payload;

	// Check if this is an FMT record
	bool is_fmt_record = (msg->payload_len >= 3 &&
	                      payload[0] == 'F' && payload[1] == 'M' && payload[2] == 'T');

	if (is_fmt_record && msg->payload_len >= 7) { // FMT + name4 minimum
		// Extract message name from FMT record (bytes 3-6)
		char name4[5] = {0};
		memcpy(name4, payload + 3, 4);

		// Calculate hash
		uint64_t hash64 = 0;
		hash_fnv_1a((uint32_t)strnlen(name4, 4), (const uint8_t*)name4, &hash64);
		uint32_t hash32 = (uint32_t)(hash64 ^ (hash64 >> 32));

		// Mark as seen in the file's hash table
		if (!logger_ensure_base_dir()) {
			LOGGER_DEBUG(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR, "no fs");
			return;
		}
		struct open_file_s* of = logger_open_or_get_file(msg->prefix, msg->suffix, msg->payload_len);
		if (of) {
			chSysLock();
			logger_fmt_seen_add(of, hash32);
			chSysUnlock();
		}
	}

	// Write the record to file (FMT or data record)
	if (!logger_ensure_base_dir()) {
		LOGGER_DEBUG(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR, "no fs");
		return;
	}
	struct open_file_s* of = logger_open_or_get_file(msg->prefix, msg->suffix, msg->payload_len);
	if (!of) {
		LOGGER_DEBUG(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR, "open fail %s.%s", msg->prefix, msg->suffix);
		return;
	}
	logger_file_write_record(of, msg->payload, msg->payload_len);
}

RUN_ON(PUBSUB_TOPIC_INIT) {
#ifdef LOGGER_PUBSUB_TOPIC_GROUP
	pubsub_init_topic(&log_msg_topic, &LOGGER_PUBSUB_TOPIC_GROUP);
#else
	pubsub_init_topic(&log_msg_topic, NULL);
#endif
	worker_thread_add_listener_task(&WT, &log_msg_listener_task, &log_msg_topic, log_msg_handler, NULL);
	logger_ensure_base_dir();
	LOGGER_DEBUG(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO, "ready");
}

static void logger_header_payload_writer(size_t sz, void* msg, void* c) {
	(void)sz;
	struct { const struct logger_msg_s* hdr; const void* payload; }* pc = c;
	memcpy(msg, pc->hdr, sizeof(struct logger_msg_s));
	memcpy((uint8_t*)msg + sizeof(struct logger_msg_s), pc->payload, pc->hdr->payload_len);
}

void logger_write(const char* fileprefix, const char* filesuffix, const void* data, size_t data_len) {
	if (!fileprefix || !filesuffix || !data || data_len == 0) {
		return;
	}
	struct {
		struct logger_msg_s hdr;
		uint8_t payload[0];
	} __attribute__((packed)) *tmp;
	(void)tmp;
	struct logger_msg_s header;
	memset(&header, 0, sizeof(header));
	strncpy(header.prefix, fileprefix, LOGGER_MAX_PREFIX_LEN);
	strncpy(header.suffix, filesuffix, LOGGER_MAX_SUFFIX_LEN);
	header.payload_len = (uint32_t)data_len;
	struct {
		const struct logger_msg_s* hdr;
		const void* payload;
	} pack_ctx = { &header, data };
	pubsub_publish_message(&log_msg_topic, sizeof(struct logger_msg_s) + data_len, logger_header_payload_writer, &pack_ctx);
}

bool logger_write_I(const char* fileprefix, const char* filesuffix, const void* data, size_t data_len) {
	if (!fileprefix || !filesuffix || !data || data_len == 0) {
		return false;
	}
	struct logger_msg_s header;
	memset(&header, 0, sizeof(header));
	strncpy(header.prefix, fileprefix, LOGGER_MAX_PREFIX_LEN);
	strncpy(header.suffix, filesuffix, LOGGER_MAX_SUFFIX_LEN);
	header.payload_len = (uint32_t)data_len;
	struct {
		const struct logger_msg_s* hdr;
		const void* payload;
	} pack_ctx = { &header, data };
	return pubsub_try_publish_message_I(&log_msg_topic, sizeof(struct logger_msg_s) + data_len, logger_header_payload_writer, &pack_ctx);
}

#ifndef LOGGER_FMT_SET_SIZE
#define LOGGER_FMT_SET_SIZE 16
#endif

static bool logger_fmt_seen_add(struct open_file_s* f, uint32_t name_hash) {
	for (size_t i=0; i<LOGGER_FMT_SET_SIZE; i++) {
		if (f->fmt_hashes[i] == name_hash) {
			return true;
		}
	}
	for (size_t i=0; i<LOGGER_FMT_SET_SIZE; i++) {
		if (f->fmt_hashes[i] == 0) {
			f->fmt_hashes[i] = name_hash;
			return false;
		}
	}
	return true;
}

static void logger_pack_and_write_ap(const char* prefix, const char* suffix, const char* name, const char* format, const char* labels, va_list ap) {
	if (!name || !format || !labels) {
		return;
	}
	char name4[4] = {0,0,0,0};
	strncpy(name4, name, 4);
	uint64_t hash64 = 0;
	hash_fnv_1a((uint32_t)strnlen(name4,4), (const uint8_t*)name4, &hash64);
	uint32_t hash32 = (uint32_t)(hash64 ^ (hash64>>32));

	struct open_file_s* of = logger_open_or_get_file(prefix, suffix, 0);
	if (!of) {
		return;
	}
	bool already_seen = logger_fmt_seen_add(of, hash32);
	if (!already_seen) {
		char fmt_hdr[3] = { 'F','M','T' };
		size_t fmt_len = strlen(format)+1;
		size_t labels_len = strlen(labels)+1;
		size_t payload_len = sizeof(fmt_hdr) + sizeof(name4) + fmt_len + labels_len;
		uint8_t* payload = chCoreAlloc(payload_len);
		if (payload) {
			uint8_t* p = payload;
			memcpy(p, fmt_hdr, sizeof(fmt_hdr)); p += sizeof(fmt_hdr);
			memcpy(p, name4, sizeof(name4)); p += sizeof(name4);
			memcpy(p, format, fmt_len); p += fmt_len;
			memcpy(p, labels, labels_len);
			logger_file_write_record(of, payload, (uint32_t)payload_len);
		}
	}

	size_t packed_size = sizeof(name4);
	for (const char* f = format; *f; f++) {
		switch (*f) {
			case 'b': case 'B': packed_size += 1; break;
			case 'h': case 'H': packed_size += 2; break;
			case 'i': case 'I': packed_size += 4; break;
			case 'L': packed_size += 4; break;
			case 'q': case 'Q': packed_size += 8; break;
			case 'f': packed_size += 4; break;
			case 'd': packed_size += 8; break;
			default: break;
		}
	}
	uint8_t* payload = chCoreAlloc(packed_size);
	if (!payload) {
		return;
	}
	uint8_t* p = payload;
	memcpy(p, name4, sizeof(name4));
	p += sizeof(name4);
	for (const char* f = format; *f; f++) {
		switch (*f) {
			case 'b': { int v = va_arg(ap, int); int8_t x = (int8_t)v; memcpy(p, &x, 1); p += 1; break; }
			case 'B': { int v = va_arg(ap, int); uint8_t x = (uint8_t)v; memcpy(p, &x, 1); p += 1; break; }
			case 'h': { int v = va_arg(ap, int); int16_t x = (int16_t)v; memcpy(p, &x, 2); p += 2; break; }
			case 'H': { int v = va_arg(ap, int); uint16_t x = (uint16_t)v; memcpy(p, &x, 2); p += 2; break; }
			case 'i': { int32_t x = va_arg(ap, int32_t); memcpy(p, &x, 4); p += 4; break; }
			case 'I': { uint32_t x = va_arg(ap, uint32_t); memcpy(p, &x, 4); p += 4; break; }
			case 'L': { uint32_t x = va_arg(ap, uint32_t); memcpy(p, &x, 4); p += 4; break; }
			case 'q': { int64_t x = va_arg(ap, int64_t); memcpy(p, &x, 8); p += 8; break; }
			case 'Q': { uint64_t x = va_arg(ap, uint64_t); memcpy(p, &x, 8); p += 8; break; }
			case 'f': { double v = va_arg(ap, double); float x = (float)v; memcpy(p, &x, 4); p += 4; break; }
			case 'd': { double x = va_arg(ap, double); memcpy(p, &x, 8); p += 8; break; }
			default: { (void)va_arg(ap, int); break; }
		}
	}
	logger_file_write_record(of, payload, (uint32_t)packed_size);
}

void logger_write_ap(const char* fileprefix, const char* filesuffix, const char* name, const char* format, const char* labels, ...) {
	va_list ap;
	va_start(ap, labels);
	logger_pack_and_write_ap(fileprefix, filesuffix, name, format, labels, ap);
	va_end(ap);
}

#define LOGGER_AP_MAX_PAYLOAD_SIZE 256
static uint8_t logger_ap_payload_buffer[LOGGER_AP_MAX_PAYLOAD_SIZE];

#define LOGGER_FMT_MAX_PAYLOAD_SIZE 128
static uint8_t logger_fmt_payload_buffer[LOGGER_FMT_MAX_PAYLOAD_SIZE];

static void logger_pack_ap_payload(uint8_t* payload, size_t* payload_size, const char* name, const char* format, va_list ap) {
	if (!name || !format || !payload || !payload_size) {
		*payload_size = 0;
		return;
	}

	char name4[4] = {0,0,0,0};
	strncpy(name4, name, 4);
	uint8_t* p = payload;
	size_t max_size = *payload_size;

	// Reserve space for name4
	if (max_size < sizeof(name4)) {
		*payload_size = 0;
		return;
	}
	memcpy(p, name4, sizeof(name4));
	p += sizeof(name4);
	size_t used = sizeof(name4);

	// Calculate total packed size first
	size_t packed_size = sizeof(name4);
	for (const char* f = format; *f; f++) {
		switch (*f) {
			case 'b': case 'B': packed_size += 1; break;
			case 'h': case 'H': packed_size += 2; break;
			case 'i': case 'I': packed_size += 4; break;
			case 'L': packed_size += 4; break;
			case 'q': case 'Q': packed_size += 8; break;
			case 'f': packed_size += 4; break;
			case 'd': packed_size += 8; break;
			default: break;
		}
	}

	if (packed_size > max_size) {
		*payload_size = 0;
		return;
	}

	// Pack the data
	for (const char* f = format; *f; f++) {
		switch (*f) {
			case 'b': { int v = va_arg(ap, int); int8_t x = (int8_t)v; memcpy(p, &x, 1); p += 1; used += 1; break; }
			case 'B': { int v = va_arg(ap, int); uint8_t x = (uint8_t)v; memcpy(p, &x, 1); p += 1; used += 1; break; }
			case 'h': { int v = va_arg(ap, int); int16_t x = (int16_t)v; memcpy(p, &x, 2); p += 2; used += 2; break; }
			case 'H': { int v = va_arg(ap, int); uint16_t x = (uint16_t)v; memcpy(p, &x, 2); p += 2; used += 2; break; }
			case 'i': { int32_t x = va_arg(ap, int32_t); memcpy(p, &x, 4); p += 4; used += 4; break; }
			case 'I': { uint32_t x = va_arg(ap, uint32_t); memcpy(p, &x, 4); p += 4; used += 4; break; }
			case 'L': { uint32_t x = va_arg(ap, uint32_t); memcpy(p, &x, 4); p += 4; used += 4; break; }
			case 'q': { int64_t x = va_arg(ap, int64_t); memcpy(p, &x, 8); p += 8; used += 8; break; }
			case 'Q': { uint64_t x = va_arg(ap, uint64_t); memcpy(p, &x, 8); p += 8; used += 8; break; }
			case 'f': { double v = va_arg(ap, double); float x = (float)v; memcpy(p, &x, 4); p += 4; used += 4; break; }
			case 'd': { double x = va_arg(ap, double); memcpy(p, &x, 8); p += 8; used += 8; break; }
			default: { (void)va_arg(ap, int); break; }
		}
	}

	*payload_size = used;
}

bool logger_generate_fmt_I(const char* fileprefix, const char* filesuffix, const char* name, const char* format, const char* labels) {
	if (!name || !format || !labels) {
		return false;
	}

	char name4[4] = {0,0,0,0};
	strncpy(name4, name, 4);

	// Generate FMT record payload
	char fmt_hdr[3] = { 'F','M','T' };
	size_t fmt_len = strlen(format) + 1;
	size_t labels_len = strlen(labels) + 1;
	size_t payload_len = sizeof(fmt_hdr) + sizeof(name4) + fmt_len + labels_len;

	if (payload_len > LOGGER_FMT_MAX_PAYLOAD_SIZE) {
		return false;
	}

	uint8_t* payload = logger_fmt_payload_buffer;
	uint8_t* p = payload;

	memcpy(p, fmt_hdr, sizeof(fmt_hdr)); p += sizeof(fmt_hdr);
	memcpy(p, name4, sizeof(name4)); p += sizeof(name4);
	memcpy(p, format, fmt_len); p += fmt_len;
	memcpy(p, labels, labels_len);

	// Send FMT record over pubsub
	struct logger_msg_s header;
	memset(&header, 0, sizeof(header));
	strncpy(header.prefix, fileprefix, LOGGER_MAX_PREFIX_LEN);
	strncpy(header.suffix, filesuffix, LOGGER_MAX_SUFFIX_LEN);
	header.payload_len = (uint32_t)payload_len;

	struct {
		const struct logger_msg_s* hdr;
		const void* payload;
	} pack_ctx = { &header, payload };

	return pubsub_try_publish_message_I(&log_msg_topic, sizeof(struct logger_msg_s) + payload_len, logger_header_payload_writer, &pack_ctx);
}

bool logger_write_ap_I(const char* fileprefix, const char* filesuffix, const char* name, const char* format, ...) {
	if (!fileprefix || !filesuffix || !name || !format) {
		return false;
	}

	va_list ap;
	va_start(ap, format);

	size_t payload_size = LOGGER_AP_MAX_PAYLOAD_SIZE;
	logger_pack_ap_payload(logger_ap_payload_buffer, &payload_size, name, format, ap);

	va_end(ap);

	if (payload_size == 0) {
		return false;
	}

	struct logger_msg_s header;
	memset(&header, 0, sizeof(header));
	strncpy(header.prefix, fileprefix, LOGGER_MAX_PREFIX_LEN);
	strncpy(header.suffix, filesuffix, LOGGER_MAX_SUFFIX_LEN);
	header.payload_len = (uint32_t)payload_size;

	struct {
		const struct logger_msg_s* hdr;
		const void* payload;
	} pack_ctx = { &header, logger_ap_payload_buffer };

	return pubsub_try_publish_message_I(&log_msg_topic, sizeof(struct logger_msg_s) + payload_size, logger_header_payload_writer, &pack_ctx);
}
