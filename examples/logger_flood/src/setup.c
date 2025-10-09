#include <modules/worker_thread/worker_thread.h>
#include <modules/logger/logger.h>
#include <hal.h>
#include <modules/pubsub/pubsub.h>
#include <modules/can/can.h>
#include <modules/uavcan/uavcan.h>

WORKER_THREAD_TAKEOVER_MAIN(default_worker_thread, NORMALPRIO)

PUBSUB_TOPIC_GROUP_CREATE(default_topic_group, 4096)

static GPTConfig gptcfg = {
	.frequency = 1000000,
	.callback = NULL,
	.cr2 = 0,
	.dier = 0
};

static void gpt_cb(GPTDriver *gptp) {
	(void)gptp;
	uint32_t v = chSysGetRealtimeCounterX();

    chSysLockFromISR();
    logger_write_ap_I("FLOOD", "BIN", "TICK", "I", "tick", v);
    chSysUnlockFromISR();
}

RUN_AFTER(CH_SYS_INIT) {
	#if !HAL_USE_GPT
	#error HAL_USE_GPT must be enabled for logger_flood example
	#endif

	gptcfg.callback = gpt_cb;
	gptStart(&GPTD3, &gptcfg);
	gptStartContinuous(&GPTD3, 999); // 1 KHz ISR
}

WORKER_THREAD_PERIODIC_TIMER_TASK_AUTOSTART(flood_task, &default_worker_thread, chTimeMS2I(1)) {
    static uint32_t ctr;
    ctr++;
    logger_write_ap("FLOOD", "BIN", "CNT ", "I", "ctr", ctr);
}


