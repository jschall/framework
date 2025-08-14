#pragma once

//
// Configure worker threads for max_modules
// All modules run on the app's default worker thread to keep the setup simple.
// The app spawns `default_worker_thread` in src/main.c.
//

#define UAVCAN_FILE_SERVER_WORKER_THREAD             default_worker_thread
#define SENSOR_UAVCAN_IN_WORKER_THREAD               default_worker_thread
#define SENSOR_UAVCAN_OUT_WORKER_THREAD              default_worker_thread
#define TIMING_WORKER_THREAD                          default_worker_thread
#define STACK_MEASUREMENT_WORKER_THREAD               default_worker_thread
#define LOAD_MEASUREMENT_WORKER_THREAD                default_worker_thread
#define PUBSUB_MISS_MEASUREMENT_WORKER_THREAD         default_worker_thread

// CAN and UAVCAN stacks
#define CAN_TRX_WORKER_THREAD                         default_worker_thread
#define CAN_EXPIRE_WORKER_THREAD                      default_worker_thread
#define UAVCAN_RX_WORKER_THREAD                        default_worker_thread
#define UAVCAN_NODESTATUS_PUBLISHER_WORKER_THREAD       default_worker_thread
#define UAVCAN_GETNODEINFO_SERVER_WORKER_THREAD         default_worker_thread
#define UAVCAN_BEGINFIRMWAREUPDATE_SERVER_WORKER_THREAD default_worker_thread
#define UAVCAN_ALLOCATEE_WORKER_THREAD                 default_worker_thread
#define UAVCAN_RESTART_WORKER_THREAD                   default_worker_thread
#define UAVCAN_TIMESYNC_WORKER_THREAD                  default_worker_thread
#define UAVCAN_NODE_REGISTRY_WORKER_THREAD             default_worker_thread
#define CAN_AUTOBAUD_WORKER_THREAD                     default_worker_thread
#define UAVCAN_PARAM_INTERFACE_WORKER_THREAD           default_worker_thread

// Misc modules
#define USB_SLCAN_WORKER_THREAD                        default_worker_thread
#define DATALOGGER_WORKER_THREAD                        default_worker_thread
#define PIN_CHANGE_PUBLISHER_WORKER_THREAD             default_worker_thread

//
// Configure topic groups
//
#define PUBSUB_DEFAULT_TOPIC_GROUP default_topic_group

//
// Misc configs
//
#define REQUIRED_RAM_MARGIN_AFTER_INIT      512

//
// Configure debug checks
//
#define CH_DBG_SYSTEM_STATE_CHECK           TRUE
#define CH_DBG_ENABLE_CHECKS                TRUE
#define CH_DBG_ENABLE_ASSERTS               TRUE
#define CH_DBG_ENABLE_STACK_CHECK           TRUE


