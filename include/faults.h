#pragma once

#include <stdbool.h>
#include <stdint.h>

// Severity levels map 1:1 to UAVCAN NodeStatus health values
typedef enum {
    FAULT_SEVERITY_OK = 0,
    FAULT_SEVERITY_WARNING = 1,
    FAULT_SEVERITY_ERROR = 2,
    FAULT_SEVERITY_CRITICAL = 3,
} fault_severity_t;

// Returns worst active fault severity across all flags (OK if none active)
uint8_t fault_get_severity(void);

// Returns true if the named fault is active (not expired). Optionally outputs its current severity
bool fault_is_active(const char* name);
uint8_t fault_get_flag_severity(const char* name);

// Set/clear/change severity
void fault_set(const char* name, uint8_t severity, const char* reason);
void fault_set_I(const char* name, uint8_t severity, const char* reason);
void fault_set_timeout(const char* name, uint8_t severity, uint32_t timeout_ms, const char* reason);
void fault_clear(const char* name);
void fault_set_severity(const char* name, uint8_t severity);

// Optional reason message support (legacy APIs)
void fault_set_with_reason(const char* name, uint8_t severity, const char* reason);
void fault_set_timeout_with_reason(const char* name, uint8_t severity, uint32_t timeout_ms, const char* reason);
void fault_set_reason(const char* name, const char* reason);
const char* fault_get_reason(const char* name);


