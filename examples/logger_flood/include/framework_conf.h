#pragma once

#define PUBSUB_DEFAULT_TOPIC_GROUP default_topic_group

#define LOGGER_WORKER_THREAD default_worker_thread

#define TIMING_WORKER_THREAD default_worker_thread

#define CAN_TRX_WORKER_THREAD                         default_worker_thread
#define CAN_EXPIRE_WORKER_THREAD                      default_worker_thread
#define UAVCAN_RX_WORKER_THREAD                        default_worker_thread
#define UAVCAN_NODESTATUS_PUBLISHER_WORKER_THREAD       default_worker_thread
#define UAVCAN_GETNODEINFO_SERVER_WORKER_THREAD         default_worker_thread
#define UAVCAN_BEGINFIRMWAREUPDATE_SERVER_WORKER_THREAD default_worker_thread
#define UAVCAN_ALLOCATEE_WORKER_THREAD                 default_worker_thread
#define UAVCAN_RESTART_WORKER_THREAD                   default_worker_thread
#define UAVCAN_TIMESYNC_WORKER_THREAD                  default_worker_thread
#define UAVCAN_PARAM_INTERFACE_WORKER_THREAD           default_worker_thread
#define CAN_AUTOBAUD_WORKER_THREAD                      default_worker_thread
#define FAULTS_WORKER_THREAD                             default_worker_thread

// Severity for uSDfault when filesystem is unavailable
#ifndef USD_FAULT_SEVERITY
#define USD_FAULT_SEVERITY FAULT_SEVERITY_ERROR
#endif

#ifndef HAL_USE_SDC
#define HAL_USE_SDC                          TRUE
#endif
#ifndef STM32_SDC_USE_SDMMC1
#define STM32_SDC_USE_SDMMC1                 TRUE
#endif


