#pragma once

#include <stddef.h>
#include <stdbool.h>

// Logger public API
//
// Overview
// - All public writes enqueue messages via pubsub. Only the logger worker
//   thread touches filesystem state and DataFlash framing/mapping.
// - AP-style writes (logger_write_ap / logger_write_ap_I) auto-emit a single
//   FMT per message name per file and frame records as ArduPilot DataFlash.
//
// Files & rotation
// - Files are created under LOGGER_BASE_DIR (default: "/LOGS") with
//   <prefix>_<index>.<suffix>. Rotation and free-space checks happen inside
//   the worker and include DataFlash header/CRC overhead.
//
// Filenames are created under LOGGER_BASE_DIR (default: "/LOGS") as:
//   <prefix>_<index>.<suffix>
// where <index> is an auto-incrementing positive integer (starting at 1).
//
// Constraints:
// - prefix: up to 8 characters (8.3 filename compatibility)
// - suffix: up to 3 characters (no leading dot)
//
// The write is enqueued; data is copied into an internal message buffer.
// Thread-safe; may block briefly.
void logger_write(const char* fileprefix, const char* filesuffix, const void* data, size_t data_len);

// ISR-safe, non-blocking variant using pubsub_try_publish_message_I.
// Returns false if it could not enqueue. No heap allocations.
bool logger_write_I(const char* fileprefix, const char* filesuffix, const void* data, size_t data_len);

// ArduPilot-like logging API.
// When a message name is first seen per logfile, a DataFlash FMT record is
// automatically written describing the format and labels. Subsequent calls
// write framed records encoded according to the format string.
// labels: comma-separated column names (e.g. "TimeUS,Roll,Pitch")
// Thread-safe; may block briefly.
void logger_write_ap(const char* fileprefix,
			  const char* filesuffix,
			  const char* name,
			  const char* format,
			  const char* labels,
			  ...);

// ISR-safe, non-blocking variant. Accepts labels and will publish an FMT
// message on first use (deduplicated in the worker). No heap allocations.
bool logger_write_ap_I(const char* fileprefix,
                      const char* filesuffix,
                      const char* name,
                      const char* format,
                      const char* labels,
                      ...);

// Optional configuration macros (override in framework_conf.h):
//   #define LOGGER_WORKER_THREAD           my_worker_thread
//   #define LOGGER_PUBSUB_TOPIC_GROUP      my_topic_group
//   #define LOGGER_BASE_DIR                "/LOGS"
//   #define LOGGER_ROTATE_BYTES            (1024*1024)
//   #define LOGGER_MIN_FREE_BYTES          (8*1024*1024)
//   #define LOGGER_MAX_PREFIX_LEN          8
//   #define LOGGER_MAX_SUFFIX_LEN          3

