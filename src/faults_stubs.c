#include <faults.h>

#ifndef MODULE_FAULTS_ENABLED

uint8_t fault_get_severity(void) { return 0; }
bool fault_is_active(const char* name) { (void)name; return false; }
uint8_t fault_get_flag_severity(const char* name) { (void)name; return 0; }
void fault_set(const char* name, uint8_t severity, const char* reason) { (void)name; (void)severity; (void)reason; }
void fault_set_timeout(const char* name, uint8_t severity, uint32_t timeout_ms, const char* reason) { (void)name; (void)severity; (void)timeout_ms; (void)reason; }
void fault_clear(const char* name) { (void)name; }
void fault_set_severity(const char* name, uint8_t severity) { (void)name; (void)severity; }
void fault_set_with_reason(const char* name, uint8_t severity, const char* reason) { (void)name; (void)severity; (void)reason; }
void fault_set_timeout_with_reason(const char* name, uint8_t severity, uint32_t timeout_ms, const char* reason) { (void)name; (void)severity; (void)timeout_ms; (void)reason; }
void fault_set_reason(const char* name, const char* reason) { (void)name; (void)reason; }
const char* fault_get_reason(const char* name) { (void)name; return NULL; }

#endif


