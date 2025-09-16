#include "logger.h"
#include <hal.h>

#include "fatfs/ff.h"

#include <modules/worker_thread/worker_thread.h>
#include <modules/pubsub/pubsub.h>

#ifndef LOGGER_WORKER_THREAD
#error Please define LOGGER_WORKER_THREAD in framework_conf.h.
#endif

#define WT LOGGER_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

static FATFS SDC_FS;

static struct pubsub_topic_s log_msg_topic;
static struct worker_thread_listener_task_s log_msg_listener_task;
static void log_msg_handler(size_t msg_size, const void* buf, void* ctx);

RUN_ON(PUBSUB_TOPIC_INIT) {
    pubsub_init_topic(&log_msg_topic, NULL);
    worker_thread_add_listener_task(&WT, &log_msg_listener_task, &log_msg_topic, log_msg_handler, NULL);

    sdcStart(&SDCD1, NULL);
    if (sdcConnect(&SDCD1) != HAL_SUCCESS) {
        return;
    }

    f_mount(&SDC_FS, "/", 1);

    FRESULT res;

    res = f_mkdir("/CUBEPILOT");
}

static void log_msg_handler(size_t msg_size, const void* buf, void* ctx) {

}
