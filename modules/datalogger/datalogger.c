#include "datalogger.h"

#include <common/slip.h>
#include <common/helpers.h>
#include <modules/uavcan_debug/uavcan_debug.h>

#ifndef DATALOGGER_WORKER_THREAD
#error Please define DATALOGGER_WORKER_THREAD in framework_conf.h.
#endif

#define WT DATALOGGER_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

MEMORYPOOL_DECL(log_writer_pool, sizeof(struct log_writer_s), PORT_NATURAL_ALIGN, chCoreAllocAlignedI);

static FATFS* fatfs;
static void log_msg_handler(size_t msg_size, const void* buf, void* ctx);

RUN_ON(PUBSUB_TOPIC_INIT) {
    fatfs = uSD_get_filesystem();
}

void logger_start_log(struct logfile_s* logfile, const char* name) {
    logfile->writer_list_head = NULL;
    logfile->open_res = f_open(&logfile->fp, name, FA_CREATE_ALWAYS | FA_WRITE);
    f_truncate(&logfile->fp);

}

void logger_subscribe(struct logfile_s* logfile, struct pubsub_topic_s* topic) {
    struct log_writer_s* writer = chPoolAlloc(&log_writer_pool);
    writer->logfile = logfile;

    LINKED_LIST_APPEND(struct log_writer_s, logfile->writer_list_head, writer);

    worker_thread_add_listener_task(&WT, &writer->listener_task, topic, log_msg_handler, writer);
}

static void log_msg_handler(size_t msg_size, const void* buf, void* ctx) {
    struct log_writer_s* writer = ctx;
//     uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "sub", "type %u %08X", *(unsigned*)buf, (unsigned)buf);


    uint16_t crc16 = crc16_ccitt(buf, msg_size, 0);

    UINT bw;
    f_write(&writer->logfile->fp, &msg_size, sizeof(msg_size), &bw);
    f_write(&writer->logfile->fp, &crc16, 2, &bw);
    f_write(&writer->logfile->fp, buf, msg_size, &bw);




//     const uint8_t* next_byte = buf;
//
//     while (msg_size != 0) {
//         uint8_t encoded[128];
//         uint8_t encoded_len = 0;
//         while (msg_size != 0 && slip_encode_and_append(*next_byte, &encoded_len, &encoded[encoded_len], 128-encoded_len)) {
//             next_byte++;
//             msg_size--;
//         }
//         res = f_write(&writer->logfile->fp, encoded, encoded_len, &bw);
//     }
//     res = f_write(&writer->logfile->fp, "\xC0", 1, &bw);
//     res = f_sync(&writer->logfile->fp);

}
