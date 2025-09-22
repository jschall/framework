#include <modules/worker_thread/worker_thread.h>
#include <modules/pubsub/pubsub.h>

PUBSUB_TOPIC_GROUP_CREATE(default_topic_group, 16384)
PUBSUB_TOPIC_GROUP_CREATE(can_topic_group, 16384)


WORKER_THREAD_TAKEOVER_MAIN(lpwork_thread, LOWPRIO+1)
WORKER_THREAD_SPAWN(slcan_thread, LOWPRIO+4, 2048)
WORKER_THREAD_SPAWN(can_thread, LOWPRIO+2, 2048)
WORKER_THREAD_SPAWN(led_thread, LOWPRIO+3, 2048)
