#include <ch.h>
#include <hal.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/pubsub/pubsub.h>

// Create default worker thread and pubsub group used by macros expecting default names.
WORKER_THREAD_SPAWN(default_worker_thread, NORMALPRIO, 2048);
PUBSUB_TOPIC_GROUP_CREATE(default_topic_group, 1024);

int main(void) {
	// ChibiOS HAL and kernel init
	halInit();
	chSysInit();

    // Nothing else to do; worker thread runs tasks via INIT hooks.

	while (true) {
		chThdSleepMilliseconds(1000);
	}
}


