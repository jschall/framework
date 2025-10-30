#include "logger.h"
#include <hal.h>
#include <stdarg.h>
#include <string.h>
#include <chprintf.h>
#include <math.h>

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

/* Rate-limited debug logging to avoid flooding under fault conditions (e.g., no SD card). */
#ifdef MODULE_UAVCAN_DEBUG_ENABLED
#define __LOG_CONCAT_IMPL(a, b) a##b
#define __LOG_CONCAT(a, b) __LOG_CONCAT_IMPL(a, b)
#define LOGGER_DEBUG_RL(level, interval_ms, fmt, ...) do { \
	static systime_t __LOG_CONCAT(_last_log_ts_, __LINE__) = 0; \
	if (chVTTimeElapsedSinceX(__LOG_CONCAT(_last_log_ts_, __LINE__)) >= chTimeMS2I(interval_ms)) { \
		__LOG_CONCAT(_last_log_ts_, __LINE__) = chVTGetSystemTimeX(); \
		LOGGER_DEBUG(level, fmt, ##__VA_ARGS__); \
	} \
} while (0)
#else
#define LOGGER_DEBUG_RL(level, interval_ms, fmt, ...) do { (void)0; } while (0)
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

// ArduPilot binary log format constants
#define AP_HEAD_BYTE1 0xA3
#define AP_HEAD_BYTE2 0x95
#define AP_FMT_MSG_ID 0x80

// Maximum lengths for FMT message fields
#define AP_MAX_NAME_LEN 4
#define AP_MAX_FORMAT_LEN 16
#define AP_MAX_LABELS_LEN 64

// Maximum number of message types per file
#define AP_MAX_MESSAGE_TYPES 256


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
	/* Buffered write: accumulate data; flush only in 512-byte aligned multiples */
	uint8_t writebuf[16384];
	uint16_t writebuf_len;
	struct open_file_s* next;
};

MEMORYPOOL_DECL(logger_open_file_pool, sizeof(struct open_file_s), PORT_NATURAL_ALIGN, chCoreAllocAlignedI);
static struct open_file_s* open_file_list_head;

// ArduPilot message type tracking
struct ap_message_type_s {
    char name[AP_MAX_NAME_LEN + 1];
    char format[AP_MAX_FORMAT_LEN + 1];
    char labels[AP_MAX_LABELS_LEN + 1];
    uint8_t id;
    uint8_t message_size;
    bool registered;
    uint32_t fmt_written_index; // file index for which FMT has been written
};

struct ap_file_context_s {
    char prefix[LOGGER_MAX_PREFIX_LEN + 1];
    char suffix[LOGGER_MAX_SUFFIX_LEN + 1];
    struct ap_message_type_s message_types[AP_MAX_MESSAGE_TYPES];
    uint8_t next_message_id;
    uint32_t fmt_written_index; // last file index for which FMT headers were written
    struct ap_file_context_s* next;
};

MEMORYPOOL_DECL(logger_ap_file_context_pool, sizeof(struct ap_file_context_s), PORT_NATURAL_ALIGN, chCoreAllocAlignedI);
static struct ap_file_context_s* ap_file_context_list_head;


static struct pubsub_topic_s log_msg_topic;
static struct worker_thread_listener_task_s log_msg_listener_task;
static void log_msg_handler(size_t msg_size, const void* buf, void* ctx);
static struct worker_thread_timer_task_s logger_sync_task;
static void logger_sync_task_func(struct worker_thread_timer_task_s* task);
static struct worker_thread_timer_task_s logger_idle_sync_task; // one-shot idle flush

/* Bytes-written statistics */
static uint64_t logger_bytes_written_total;
static struct worker_thread_timer_task_s logger_stats_task;
static void logger_stats_task_func(struct worker_thread_timer_task_s* task);
// Forward declaration used by logger_ensure_fmt_for_file
static struct ap_file_context_s* ap_get_or_create_file_context(const char* prefix, const char* suffix);

/* SD sector size (bytes). All physical writes must be multiples of this. */
#ifndef LOGGER_SECTOR_SIZE
#define LOGGER_SECTOR_SIZE 512u
#endif

/* Flush as many full sectors from write buffer as possible, keeping any tail (<512 bytes) buffered. */
static void logger_writebuf_flush_aligned(struct open_file_s* of, bool do_sync) {
	if (!of) {
		return;
	}
	uint32_t to_write = (uint32_t)of->writebuf_len & ~(LOGGER_SECTOR_SIZE - 1u);
	if (to_write == 0) {
		return;
	}
	uint32_t off = 0;
	while (off < to_write) {
		UINT bw = 0;
		UINT chunk = (UINT)(to_write - off);
		FRESULT wr = f_write(&of->fp, &of->writebuf[off], chunk, &bw);
		if (wr != FR_OK || bw == 0) {
			break;
		}
		off += bw;
		of->bytes_written += bw;
		logger_bytes_written_total += bw;
	}
	/* shift any remaining buffered tail to the start */
	if (off > 0) {
		uint32_t remain = (uint32_t)of->writebuf_len - off;
		if (remain > 0) {
			memmove(of->writebuf, &of->writebuf[off], remain);
		}
		of->writebuf_len = (uint16_t)remain;
		if (do_sync) {
			f_sync(&of->fp);
		}
	}
}

/* Append data to write buffer, flushing in aligned multiples if buffer is full. */
static void logger_writebuf_append(struct open_file_s* of, const uint8_t* data, uint32_t len) {
	if (!of || !data || len == 0) {
		return;
	}
	while (len > 0) {
		if (of->writebuf_len == sizeof(of->writebuf)) {
			/* attempt to flush a full buffer (which is sector-multiple sized) */
			logger_writebuf_flush_aligned(of, true);
			/* if still full, give up this cycle */
			if (of->writebuf_len == sizeof(of->writebuf)) {
				break;
			}
		}
		uint32_t space = (uint32_t)sizeof(of->writebuf) - of->writebuf_len;
		uint32_t to_copy = (len < space) ? len : space;
		memcpy(&of->writebuf[of->writebuf_len], data, to_copy);
		of->writebuf_len += (uint16_t)to_copy;
		data += to_copy;
		len -= to_copy;
	}
}

// Ensure FMT records are present for the current open file. Writes FMT directly.
static void logger_ensure_fmt_for_file(const char* prefix, const char* suffix, struct open_file_s* of) {
	struct ap_file_context_s* actx = ap_get_or_create_file_context(prefix, suffix);
	if (!actx) {
		return;
	}
	for (size_t i = 0; i < AP_MAX_MESSAGE_TYPES; i++) {
		if (actx->message_types[i].id == 0) {
			continue; // unused slot
		}
		if (actx->message_types[i].fmt_written_index == of->index) {
			continue; // already written for this file index
		}
		// Build FMT record (89 bytes)
		uint8_t fmt_data[89];
		size_t offset = 0;
		fmt_data[offset++] = AP_HEAD_BYTE1;
		fmt_data[offset++] = AP_HEAD_BYTE2;
		fmt_data[offset++] = AP_FMT_MSG_ID;
		fmt_data[offset++] = actx->message_types[i].id; // type id
		fmt_data[offset++] = actx->message_types[i].message_size; // length
		for (size_t j = 0; j < AP_MAX_NAME_LEN; j++) {
			fmt_data[offset++] = (j < strlen(actx->message_types[i].name)) ? (uint8_t)actx->message_types[i].name[j] : 0;
		}
		for (size_t j = 0; j < AP_MAX_FORMAT_LEN; j++) {
			fmt_data[offset++] = (j < strlen(actx->message_types[i].format)) ? (uint8_t)actx->message_types[i].format[j] : 0;
		}
		for (size_t j = 0; j < AP_MAX_LABELS_LEN; j++) {
			fmt_data[offset++] = (j < strlen(actx->message_types[i].labels)) ? (uint8_t)actx->message_types[i].labels[j] : 0;
		}
		/* Queue FMT record into write buffer to preserve 512-byte file alignment */
		logger_writebuf_append(of, fmt_data, (uint32_t)sizeof(fmt_data));
		actx->message_types[i].fmt_written_index = of->index;
		actx->message_types[i].registered = true;
	}
}

// ArduPilot logging helper functions
static uint8_t ap_calculate_message_size(const char* format) {
    uint8_t size = 3; // header (3 bytes)
    for (size_t i = 0; format[i] != '\0'; i++) {
        switch (format[i]) {
            case 'b': case 'B': size += 1; break; // int8/uint8
            case 'h': case 'H': size += 2; break; // int16/uint16
            case 'i': case 'I': size += 4; break; // int32/uint32
            case 'f':           size += 4; break; // float32
            case 'd':           size += 8; break; // float64
            case 'q': case 'Q': size += 8; break; // int64/uint64
            case 'n':           size += 4; break; // char[4]
            case 'N':           size += 16; break; // char[16]
            case 'Z':           size += 64; break; // char[64]
            case 'c': case 'C': size += 2; break; // int16/uint16 * 100
            case 'e': case 'E': size += 4; break; // int32/uint32 * 100
            case 'L':           size += 4; break; // GPS coordinate
            case 'M':           size += 1; break; // flight mode
            default:
                // Invalid format character - return 0 to indicate error
                return 0;
        }
    }
    return size;
}

static struct ap_file_context_s* ap_get_or_create_file_context_I(const char* prefix, const char* suffix) {
    chDbgCheckClassI();
    struct ap_file_context_s* ctx = ap_file_context_list_head;
    while (ctx) {
        if (strncmp(ctx->prefix, prefix, LOGGER_MAX_PREFIX_LEN) == 0 &&
            strncmp(ctx->suffix, suffix, LOGGER_MAX_SUFFIX_LEN) == 0) {
            return ctx;
        }
        ctx = ctx->next;
    }

    // Create new context (I-class allocation)
    ctx = chPoolAllocI(&logger_ap_file_context_pool);
    if (!ctx) {
        chPoolAddI(&logger_ap_file_context_pool, chCoreAllocAlignedI(sizeof(struct ap_file_context_s), PORT_NATURAL_ALIGN));
        ctx = chPoolAllocI(&logger_ap_file_context_pool);
    }
    if (!ctx) {
        return NULL;
    }

    memset(ctx, 0, sizeof(*ctx));
    strncpy(ctx->prefix, prefix, LOGGER_MAX_PREFIX_LEN);
    strncpy(ctx->suffix, suffix, LOGGER_MAX_SUFFIX_LEN);
    ctx->next_message_id = 1; // Reserve 0 for special cases
    ctx->fmt_written_index = 0;

    LINKED_LIST_APPEND(struct ap_file_context_s, ap_file_context_list_head, ctx);
    return ctx;
}

static struct ap_file_context_s* ap_get_or_create_file_context(const char* prefix, const char* suffix) {
    chSysLock();
    struct ap_file_context_s* ctx = ap_get_or_create_file_context_I(prefix, suffix);
    chSysUnlock();
    return ctx;
}

static struct ap_message_type_s* ap_find_or_register_message_type_I(struct ap_file_context_s* ctx, const char* name, const char* format, const char* labels) {
    chDbgCheckClassI();
    // First check if this message type already exists
    for (size_t i = 0; i < AP_MAX_MESSAGE_TYPES; i++) {
        if (ctx->message_types[i].id != 0 &&
            strncmp(ctx->message_types[i].name, name, AP_MAX_NAME_LEN) == 0) {
            // Check if format/labels match
            if (strncmp(ctx->message_types[i].format, format, AP_MAX_FORMAT_LEN) == 0 &&
                strncmp(ctx->message_types[i].labels, labels, AP_MAX_LABELS_LEN) == 0) {
                return &ctx->message_types[i];
            } else {
                // Format/labels changed - this is an error in ArduPilot format
                return NULL;
            }
        }
    }

    // Find free slot and register new message type
    for (size_t i = 0; i < AP_MAX_MESSAGE_TYPES; i++) {
        if (ctx->message_types[i].id == 0) {
            struct ap_message_type_s* mt = &ctx->message_types[i];
            memset(mt, 0, sizeof(*mt));
            strncpy(mt->name, name, AP_MAX_NAME_LEN);
            strncpy(mt->format, format, AP_MAX_FORMAT_LEN);
            strncpy(mt->labels, labels, AP_MAX_LABELS_LEN);
            mt->id = ctx->next_message_id++;
            mt->message_size = ap_calculate_message_size(format);
            if (mt->message_size == 0) {
                return NULL; // Invalid format
            }
            mt->registered = false;
            return mt;
        }
    }

    return NULL; // No free slots
}

static struct ap_message_type_s* ap_find_or_register_message_type(struct ap_file_context_s* ctx, const char* name, const char* format, const char* labels) {
    chSysLock();
    struct ap_message_type_s* mt = ap_find_or_register_message_type_I(ctx, name, format, labels);
    chSysUnlock();
    return mt;
}

    // Write FMT message for a message type
// Removed: ap_write_fmt_message; FMT emission is handled in logger thread

// Removed: ap_write_fmt_message_I; FMT emission is handled in logger thread

// Encode data according to format string into binary buffer
static size_t ap_encode_message_data(uint8_t* buffer, size_t buffer_size, const char* format, va_list args) {
    size_t offset = 3; // Skip header (written by caller)

    for (size_t i = 0; format[i] != '\0'; i++) {
        if (offset >= buffer_size) {
            return 0; // Buffer overflow
        }

        switch (format[i]) {
            case 'b': { // int8
                int8_t value = (int8_t)va_arg(args, int);
                buffer[offset++] = (uint8_t)value;
                break;
            }
            case 'B': { // uint8
                uint8_t value = (uint8_t)va_arg(args, unsigned int);
                buffer[offset++] = value;
                break;
            }
            case 'h': { // int16
                int16_t value = (int16_t)va_arg(args, int);
                buffer[offset++] = (uint8_t)(value & 0xFF);
                buffer[offset++] = (uint8_t)((value >> 8) & 0xFF);
                break;
            }
            case 'H': { // uint16
                uint16_t value = (uint16_t)va_arg(args, unsigned int);
                buffer[offset++] = (uint8_t)(value & 0xFF);
                buffer[offset++] = (uint8_t)((value >> 8) & 0xFF);
                break;
            }
            case 'i': { // int32
                int32_t value = va_arg(args, int32_t);
                buffer[offset++] = (uint8_t)(value & 0xFF);
                buffer[offset++] = (uint8_t)((value >> 8) & 0xFF);
                buffer[offset++] = (uint8_t)((value >> 16) & 0xFF);
                buffer[offset++] = (uint8_t)((value >> 24) & 0xFF);
                break;
            }
            case 'I': { // uint32
                uint32_t value = va_arg(args, uint32_t);
                buffer[offset++] = (uint8_t)(value & 0xFF);
                buffer[offset++] = (uint8_t)((value >> 8) & 0xFF);
                buffer[offset++] = (uint8_t)((value >> 16) & 0xFF);
                buffer[offset++] = (uint8_t)((value >> 24) & 0xFF);
                break;
            }
            case 'f': { // float32
                float value = (float)va_arg(args, double);
                uint32_t int_value;
                memcpy(&int_value, &value, sizeof(float));
                buffer[offset++] = (uint8_t)(int_value & 0xFF);
                buffer[offset++] = (uint8_t)((int_value >> 8) & 0xFF);
                buffer[offset++] = (uint8_t)((int_value >> 16) & 0xFF);
                buffer[offset++] = (uint8_t)((int_value >> 24) & 0xFF);
                break;
            }
            case 'd': { // float64
                double value = va_arg(args, double);
                uint64_t int_value;
                memcpy(&int_value, &value, sizeof(double));
                for (size_t j = 0; j < 8; j++) {
                    buffer[offset++] = (uint8_t)(int_value & 0xFF);
                    int_value >>= 8;
                }
                break;
            }
            case 'q': { // int64
                int64_t value = va_arg(args, int64_t);
                for (size_t j = 0; j < 8; j++) {
                    buffer[offset++] = (uint8_t)(value & 0xFF);
                    value >>= 8;
                }
                break;
            }
            case 'Q': { // uint64
                uint64_t value = va_arg(args, uint64_t);
                for (size_t j = 0; j < 8; j++) {
                    buffer[offset++] = (uint8_t)(value & 0xFF);
                    value >>= 8;
                }
                break;
            }
            case 'n': case 'N': case 'Z': { // strings
                const char* str = va_arg(args, const char*);
                size_t max_len = (format[i] == 'n') ? 4 : (format[i] == 'N') ? 16 : 64;
                for (size_t j = 0; j < max_len; j++) {
                    buffer[offset++] = (j < strlen(str)) ? (uint8_t)str[j] : 0;
                }
                break;
            }
            case 'c': { // int16 * 100
                float value = (float)va_arg(args, double);
                int16_t scaled = (int16_t)roundf(value * 100.0f);
                buffer[offset++] = (uint8_t)(scaled & 0xFF);
                buffer[offset++] = (uint8_t)((scaled >> 8) & 0xFF);
                break;
            }
            case 'C': { // uint16 * 100
                float value = (float)va_arg(args, double);
                uint16_t scaled = (uint16_t)roundf(value * 100.0f);
                buffer[offset++] = (uint8_t)(scaled & 0xFF);
                buffer[offset++] = (uint8_t)((scaled >> 8) & 0xFF);
                break;
            }
            case 'e': { // int32 * 100
                float value = (float)va_arg(args, double);
                int32_t scaled = (int32_t)roundf(value * 100.0f);
                buffer[offset++] = (uint8_t)(scaled & 0xFF);
                buffer[offset++] = (uint8_t)((scaled >> 8) & 0xFF);
                buffer[offset++] = (uint8_t)((scaled >> 16) & 0xFF);
                buffer[offset++] = (uint8_t)((scaled >> 24) & 0xFF);
                break;
            }
            case 'E': { // uint32 * 100
                float value = (float)va_arg(args, double);
                uint32_t scaled = (uint32_t)roundf(value * 100.0f);
                buffer[offset++] = (uint8_t)(scaled & 0xFF);
                buffer[offset++] = (uint8_t)((scaled >> 8) & 0xFF);
                buffer[offset++] = (uint8_t)((scaled >> 16) & 0xFF);
                buffer[offset++] = (uint8_t)((scaled >> 24) & 0xFF);
                break;
            }
            case 'L': { // GPS coordinate (int32 * 1e7)
                double value = va_arg(args, double);
                int32_t scaled = (int32_t)round(value * (double)10000000.0);
                buffer[offset++] = (uint8_t)(scaled & 0xFF);
                buffer[offset++] = (uint8_t)((scaled >> 8) & 0xFF);
                buffer[offset++] = (uint8_t)((scaled >> 16) & 0xFF);
                buffer[offset++] = (uint8_t)((scaled >> 24) & 0xFF);
                break;
            }
            case 'M': { // flight mode (uint8)
                uint8_t value = (uint8_t)va_arg(args, unsigned int);
                buffer[offset++] = value;
                break;
            }
            default:
                return 0; // Invalid format character
        }
    }

    return offset; // Return total size written
}

static bool logger_ensure_base_dir(void) {
	FATFS* fs = uSD_get_filesystem();
	if (!fs) {
		return false;
	}
	(void)fs;
	FRESULT res = f_mkdir(LOGGER_BASE_DIR);
	if (res != FR_OK && res != FR_EXIST) {
		LOGGER_DEBUG_RL(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR, 1000, "mkdir %s -> %u", LOGGER_BASE_DIR, (unsigned)res);
		return false;
	}
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
		FRESULT open_res = f_open(&f->fp, path, FA_CREATE_ALWAYS | FA_WRITE | FA_OPEN_APPEND);
		if (open_res != FR_OK) {
			chSysLock();
			chPoolFreeI(&logger_open_file_pool, f);
			chSysUnlock();
			LOGGER_DEBUG_RL(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR, 1000, "open %s -> %u", path, (unsigned)open_res);
			return NULL;
		}
		f->bytes_written = f->fp.fptr;
		f->writebuf_len = 0;
		LINKED_LIST_APPEND(struct open_file_s, open_file_list_head, f);
		LOGGER_DEBUG(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO, "opened %s", path);
	}

	uint64_t needed = (uint64_t)upcoming_write_len;
	if (f->bytes_written + needed > (uint64_t)LOGGER_ROTATE_BYTES) {
		/* Flush aligned multiples; postpone rotation if a partial sector remains */
		logger_writebuf_flush_aligned(f, true);
		if ((f->writebuf_len & (LOGGER_SECTOR_SIZE - 1u)) == 0u && f->writebuf_len == 0u) {
			f_close(&f->fp);
			f->index++;
			char path[64];
			logger_build_path(path, sizeof(path), prefix, f->index, suffix);
			LOGGER_DEBUG(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO, "rotate %s_%u.%s", prefix, (unsigned)f->index, suffix);
			while (logger_get_free_bytes() < LOGGER_MIN_FREE_BYTES + needed) {
				if (!logger_delete_oldest_for_prefix(prefix, suffix)) {
					break;
				}
			}
			f_open(&f->fp, path, FA_CREATE_ALWAYS | FA_WRITE | FA_OPEN_APPEND);
			f->bytes_written = f->fp.fptr;
		} else {
			/* Can't rotate cleanly yet; continue writing to current file until next sector boundary */
		}
	}

	if (logger_get_free_bytes() < LOGGER_MIN_FREE_BYTES + needed) {
		LOGGER_DEBUG_RL(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_WARNING, 5000, "low space: need %lu", (unsigned long)(LOGGER_MIN_FREE_BYTES + needed));
		while (logger_get_free_bytes() < LOGGER_MIN_FREE_BYTES + needed) {
			if (!logger_delete_oldest_for_prefix(prefix, suffix)) {
				LOGGER_DEBUG_RL(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_WARNING, 5000, "no deletable files");
				break;
			}
		}
	}

	return f;
}

static void log_msg_handler(size_t msg_size, const void* buf, void* ctx) {
	UNUSED(ctx);
	if (msg_size < sizeof(struct logger_msg_s)) {
		LOGGER_DEBUG_RL(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_WARNING, 1000, "msg too small %u", (unsigned)msg_size);
		return;
	}
	const struct logger_msg_s* msg = buf;

	// LOGGER_DEBUG(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO, "log msg recv sz=%u payload=%u", (unsigned)msg_size, (unsigned)msg->payload_len);

    // Write the raw data to file
	if (!logger_ensure_base_dir()) {
		// LOGGER_DEBUG(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR, "no fs");
		return;
	}

	struct open_file_s* of = logger_open_or_get_file(msg->prefix, msg->suffix, msg->payload_len);
	if (!of) {
		return;
	}
	// LOGGER_DEBUG(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO, "open ok %s_%u.%s bytes=%lu wb=%u", of->prefix, (unsigned)of->index, of->suffix, (unsigned long)of->bytes_written, (unsigned)of->writebuf_len);

	// Ensure FMT headers exist for this file index before appending any data
	logger_ensure_fmt_for_file(msg->prefix, msg->suffix, of);

	/* Append message payload to write buffer (aligned flushes handled internally) */
	logger_writebuf_append(of, (const uint8_t*)msg->payload, (uint32_t)msg->payload_len);

	/* schedule an idle-time flush for 500ms after the last write */
	worker_thread_timer_task_reschedule(&WT, &logger_idle_sync_task, chTimeMS2I(500));
	// LOGGER_DEBUG(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO, "idle flush scheduled wb=%u", (unsigned)of->writebuf_len);
	// if (wr != FR_OK || bw != msg->payload_len) {
	// 	LOGGER_DEBUG(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR, "write err %u bw=%u len=%u", (unsigned)wr, (unsigned)bw, (unsigned)msg->payload_len);
	// } else {
	// 	LOGGER_DEBUG(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO, "write %s_%u.%s %uB", of->prefix, (unsigned)of->index, of->suffix, (unsigned)bw);
	// }
}

RUN_ON(PUBSUB_TOPIC_INIT) {
#ifdef LOGGER_PUBSUB_TOPIC_GROUP
	pubsub_init_topic(&log_msg_topic, &LOGGER_PUBSUB_TOPIC_GROUP);
#else
	pubsub_init_topic(&log_msg_topic, NULL);
#endif
	worker_thread_add_listener_task(&WT, &log_msg_listener_task, &log_msg_topic, log_msg_handler, NULL);
	/* One-shot idle flush task; initially disabled (TIME_INFINITE). */
	worker_thread_add_timer_task(&WT, &logger_idle_sync_task, logger_sync_task_func, NULL, TIME_INFINITE, false);
	/* Periodic stats printer (every 1s) */
	worker_thread_add_timer_task(&WT, &logger_stats_task, logger_stats_task_func, NULL, chTimeS2I(5), true);
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

static void logger_sync_task_func(struct worker_thread_timer_task_s* task) {
	(void)task;
	struct open_file_s* f = open_file_list_head;
	while (f) {
		/* Flush only full 512-byte multiples; keep any tail buffered */
		uint16_t before = f->writebuf_len;
		logger_writebuf_flush_aligned(f, true);
		uint16_t after = f->writebuf_len;
		UNUSED(before);
		UNUSED(after);
		f_sync(&f->fp);
		f = f->next;
	}
}

static void logger_stats_task_func(struct worker_thread_timer_task_s* task) {
	(void)task;
	static uint64_t prev_total;
	uint64_t cur = logger_bytes_written_total;
	uint64_t delta = cur - prev_total;
	prev_total = cur;
	LOGGER_DEBUG(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO, "stats bytes_total=%lu KiB rate=%lu KiB/s",
		(unsigned long)(cur/1024ULL), (unsigned long)(delta/1024ULL));
}

bool logger_write_I(const char* fileprefix, const char* filesuffix, const void* data, size_t data_len) {
	if (!fileprefix || !filesuffix || !data || data_len == 0) {
		return false;
	}
	chDbgCheckClassI();
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



void logger_write_ap(const char* fileprefix, const char* filesuffix, const char* name, const char* format, const char* labels, ...) {
	if (!fileprefix || !filesuffix || !name || !format || !labels) {
		return;
	}

    // Get or create file context
    struct ap_file_context_s* ctx = ap_get_or_create_file_context(fileprefix, filesuffix);
	if (!ctx) {
		return;
	}

    // Defer FMT emission to thread context; non-ISR publish only packages the data

    // Find or register message type (thread context). Do NOT write FMT here; only update the table.
    struct ap_message_type_s* mt = ap_find_or_register_message_type(ctx, name, format, labels);
    if (!mt) {
        return;
    }

	// Encode the data message
	uint8_t buffer[256]; // Should be large enough for most messages
	buffer[0] = AP_HEAD_BYTE1;
	buffer[1] = AP_HEAD_BYTE2;
	buffer[2] = mt->id;

	va_list args;
	va_start(args, labels);
	size_t data_size = ap_encode_message_data(buffer, sizeof(buffer), format, args);
	va_end(args);

	if (data_size == 0) {
		return; // Encoding failed
	}

	// Write the data message
	logger_write(fileprefix, filesuffix, buffer, data_size);
}

bool logger_write_ap_I(const char* fileprefix, const char* filesuffix, const char* name, const char* format, const char* labels, ...) {
	if (!fileprefix || !filesuffix || !name || !format || !labels) {
		return false;
	}

	chDbgCheckClassI();

    // Get or create file context (I-class)
    struct ap_file_context_s* ctx = ap_get_or_create_file_context_I(fileprefix, filesuffix);
	if (!ctx) {
		return false;
	}

    // Defer FMT emission to thread context; ISR publish only packages the data

    // Find or register message type (I-class). Do NOT write FMT here; only update the table.
    struct ap_message_type_s* mt = ap_find_or_register_message_type_I(ctx, name, format, labels);
    if (!mt) {
        return false;
    }

	// Encode the data message
	uint8_t buffer[256]; // Should be large enough for most messages
	buffer[0] = AP_HEAD_BYTE1;
	buffer[1] = AP_HEAD_BYTE2;
	buffer[2] = mt->id;

	va_list args;
	va_start(args, labels);
	size_t data_size = ap_encode_message_data(buffer, sizeof(buffer), format, args);
	va_end(args);

	if (data_size == 0) {
		return false; // Encoding failed
	}

	// Write the data message (I-class path)
	return logger_write_I(fileprefix, filesuffix, buffer, data_size);
}
