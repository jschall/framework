#include <faults.h>

#include <common/ctor.h>
#include <string.h>
#include <modules/worker_thread/worker_thread.h>
#ifdef MODULE_UAVCAN_DEBUG_ENABLED
#include <modules/uavcan_debug/uavcan_debug.h>
#endif
#include <modules/uavcan_nodestatus_publisher/uavcan_nodestatus_publisher.h>

#ifndef FAULTS_WORKER_THREAD
#error Please define FAULTS_WORKER_THREAD in framework_conf.h.
#endif

#define WT FAULTS_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

typedef struct fault_flag_s {
    const char* name;
    uint8_t severity; // UAVCAN health enum value
    const char* reason; // Optional reason string (pointer, no copy)
    systime_t expiry_begin_systime; // when timeout was set
    systime_t expiry_duration_ticks; // TIME_INFINITE if none
    struct fault_flag_s* next;
} fault_flag_t;

static fault_flag_t* faults_head;
static struct worker_thread_timer_task_s faults_timer_task;

static fault_flag_t* find_flag(const char* name) {
    for (fault_flag_t* it = faults_head; it != NULL; it = it->next) {
        if (strcmp(it->name, name) == 0) return it;
    }
    return NULL;
}

static fault_flag_t* ensure_flag(const char* name) {
    fault_flag_t* f = find_flag(name);
    if (f) return f;
    f = chCoreAlloc(sizeof(fault_flag_t));
    if (!f) return NULL;
    f->name = name;
    f->severity = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    f->reason = NULL;
    f->expiry_begin_systime = 0;
    f->expiry_duration_ticks = TIME_INFINITE;
    f->next = faults_head;
    faults_head = f;
    return f;
}

static void update_node_health(void) {
    set_node_health(fault_get_severity());
}

static void faults_timer_task_func(struct worker_thread_timer_task_s* task) {
    (void)task;
    uint8_t worst = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;

    for (fault_flag_t* it = faults_head; it != NULL; it = it->next) {
        if (it->expiry_duration_ticks != TIME_INFINITE) {
            systime_t elapsed = chVTGetSystemTimeX() - it->expiry_begin_systime;
            if (elapsed >= it->expiry_duration_ticks) {
                it->severity = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
                it->expiry_duration_ticks = TIME_INFINITE;
            }
        }
        if (it->severity > worst) worst = it->severity;
    }

    set_node_health(worst);
}

uint8_t fault_get_severity(void) {
    uint8_t worst = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    for (fault_flag_t* it = faults_head; it != NULL; it = it->next) {
        if (it->expiry_duration_ticks != TIME_INFINITE) {
            systime_t elapsed = chVTGetSystemTimeX() - it->expiry_begin_systime;
            if (elapsed >= it->expiry_duration_ticks) {
                continue;
            }
        }
        if (it->severity > worst) worst = it->severity;
    }
    return worst;
}

bool fault_is_active(const char* name) {
    fault_flag_t* f = find_flag(name);
    if (!f) return false;
    if (f->expiry_duration_ticks != TIME_INFINITE) {
        systime_t elapsed = chVTGetSystemTimeX() - f->expiry_begin_systime;
        if (elapsed >= f->expiry_duration_ticks) return false;
    }
    return f->severity != UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
}

uint8_t fault_get_flag_severity(const char* name) {
    fault_flag_t* f = find_flag(name);
    if (!f) return UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    if (f->expiry_duration_ticks != TIME_INFINITE) {
        systime_t elapsed = chVTGetSystemTimeX() - f->expiry_begin_systime;
        if (elapsed >= f->expiry_duration_ticks) return UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    }
    return f->severity;
}

void fault_set(const char* name, uint8_t severity, const char* reason) {
    fault_flag_t* f = ensure_flag(name);
    if (!f) return;
    f->severity = severity;
    f->reason = reason;
    f->expiry_duration_ticks = TIME_INFINITE;
    update_node_health();
}

void fault_set_timeout(const char* name, uint8_t severity, uint32_t timeout_ms, const char* reason) {
    fault_flag_t* f = ensure_flag(name);
    if (!f) return;
    f->severity = severity;
    f->reason = reason;
    f->expiry_begin_systime = chVTGetSystemTimeX();
    f->expiry_duration_ticks = chTimeMS2I(timeout_ms);
    update_node_health();
}

void fault_clear(const char* name) {
    fault_flag_t* f = find_flag(name);
    if (!f) return;
    f->severity = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    f->reason = NULL;
    f->expiry_duration_ticks = TIME_INFINITE;
    update_node_health();
}

void fault_set_severity(const char* name, uint8_t severity) {
    fault_flag_t* f = ensure_flag(name);
    if (!f) return;
    f->severity = severity;
    update_node_health();
}

void fault_set_with_reason(const char* name, uint8_t severity, const char* reason) {
    fault_flag_t* f = ensure_flag(name);
    if (!f) return;
    f->severity = severity;
    f->reason = reason;
    f->expiry_duration_ticks = TIME_INFINITE;
    update_node_health();
}

void fault_set_timeout_with_reason(const char* name, uint8_t severity, uint32_t timeout_ms, const char* reason) {
    fault_flag_t* f = ensure_flag(name);
    if (!f) return;
    f->severity = severity;
    f->reason = reason;
    f->expiry_begin_systime = chVTGetSystemTimeX();
    f->expiry_duration_ticks = chTimeMS2I(timeout_ms);
    update_node_health();
}

void fault_set_reason(const char* name, const char* reason) {
    fault_flag_t* f = ensure_flag(name);
    if (!f) return;
    f->reason = reason;
}

const char* fault_get_reason(const char* name) {
    fault_flag_t* f = find_flag(name);
    if (!f) return NULL;
    return f->reason;
}

// Periodic fault printer using UAVCAN debug
static struct worker_thread_timer_task_s faults_printer_task;
static void faults_printer_task_func(struct worker_thread_timer_task_s* task) {
    (void)task;
#ifdef MODULE_UAVCAN_DEBUG_ENABLED
    // Print active faults and their reasons
    uint8_t worst = fault_get_severity();
    const char* worst_str = (worst == UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK) ? "OK" :
                            (worst == UAVCAN_PROTOCOL_NODESTATUS_HEALTH_WARNING) ? "WARNING" :
                            (worst == UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR) ? "ERROR" :
                            (worst == UAVCAN_PROTOCOL_NODESTATUS_HEALTH_CRITICAL) ? "CRITICAL" : "?";
    uavcan_send_debug_msg(LOG_LEVEL_INFO, "faults", "worst=%s(%u)", worst_str, (unsigned)worst);
    for (fault_flag_t* it = faults_head; it != NULL; it = it->next) {
        if (!fault_is_active(it->name)) continue;
        const char* sev_str = (it->severity == UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK) ? "OK" :
                              (it->severity == UAVCAN_PROTOCOL_NODESTATUS_HEALTH_WARNING) ? "WARNING" :
                              (it->severity == UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR) ? "ERROR" :
                              (it->severity == UAVCAN_PROTOCOL_NODESTATUS_HEALTH_CRITICAL) ? "CRITICAL" : "?";
        uavcan_send_debug_msg(LOG_LEVEL_INFO, "faults", "%s: %s(%u)%s%s",
                              it->name,
                              sev_str,
                              (unsigned)it->severity,
                              it->reason ? " - " : "",
                              it->reason ? it->reason : "");
    }
#endif
}

RUN_AFTER(INIT_END) {
    // initialize timer to check expirations and sync node health
    worker_thread_add_timer_task(&WT, &faults_timer_task, faults_timer_task_func, NULL, chTimeMS2I(1000), true);
    // add printer task (every 5s)
    worker_thread_add_timer_task(&WT, &faults_printer_task, faults_printer_task_func, NULL, chTimeMS2I(5000), true);
    // initialize node health from current faults
    update_node_health();
}


