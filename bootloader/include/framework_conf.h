#pragma once

//
// Configure worker threads
//

#define TIMING_WORKER_THREAD                            lpwork_thread
#define UAVCAN_NODESTATUS_PUBLISHER_WORKER_THREAD       lpwork_thread
#define CAN_AUTOBAUD_WORKER_THREAD                      lpwork_thread
#define UAVCAN_PARAM_INTERFACE_WORKER_THREAD            lpwork_thread
#define UAVCAN_GETNODEINFO_SERVER_WORKER_THREAD         lpwork_thread
#define UAVCAN_RESTART_WORKER_THREAD                    lpwork_thread
#define UAVCAN_BEGINFIRMWAREUPDATE_SERVER_WORKER_THREAD lpwork_thread
#define UAVCAN_ALLOCATEE_WORKER_THREAD                  lpwork_thread
#define BOOTLOADER_APP_THREAD                           lpwork_thread
#define LOGGER_WORKER_THREAD                            lpwork_thread
#define UAVCAN_TIMESYNC_WORKER_THREAD                   lpwork_thread
#define PUBSUB_MISS_MEASUREMENT_WORKER_THREAD           lpwork_thread
#define LOAD_MEASUREMENT_WORKER_THREAD                  lpwork_thread

#define USB_SLCAN_WORKER_THREAD                         can_thread
#define CAN_TRX_WORKER_THREAD                           can_thread
#define CAN_EXPIRE_WORKER_THREAD                        can_thread
#define UAVCAN_RX_WORKER_THREAD                         can_thread

//
// Configure topic groups
//

#define PUBSUB_DEFAULT_TOPIC_GROUP default_topic_group
#define CAN_RX_PUBSUB_TOPIC_GROUP can_topic_group

//
// Configure debug checks
//

#define CH_DBG_SYSTEM_STATE_CHECK           TRUE
#define CH_DBG_ENABLE_CHECKS                TRUE
#define CH_DBG_ENABLE_ASSERTS               TRUE
#define CH_DBG_ENABLE_STACK_CHECK           TRUE

#define CAN_TX_QUEUE_LEN 256
