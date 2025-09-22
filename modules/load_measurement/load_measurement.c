/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <common/ctor.h>
#include <ch.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/uavcan_debug/uavcan_debug.h>

#ifndef LOAD_MEASUREMENT_WORKER_THREAD
#error Please define LOAD_MEASUREMENT_WORKER_THREAD in framework_conf.h.
#endif

#define WT LOAD_MEASUREMENT_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

static struct worker_thread_timer_task_s load_print_task;
static void load_print_task_func(struct worker_thread_timer_task_s* task);

systime_t meas_begin_t;
systime_t idle_enter_t;
systime_t idle_total_ticks;

// IRQ accounting (cumulative ticks since boot)
volatile systime_t irq_enter_t;
volatile systime_t irq_total_ticks;
volatile uint32_t irq_nesting;

// Track whether we are currently in the idle thread (set in idle hooks)
volatile uint32_t idle_active;

// Track which context the outermost IRQ was entered from (idle or thread)
volatile uint32_t irq_entered_from_idle;

// Separate cumulative IRQ time buckets
volatile systime_t irq_in_idle_total_ticks;
volatile systime_t irq_in_thread_total_ticks;

// Previous snapshots for periodic delta computation
static systime_t prev_irq_in_idle_total_ticks;
static systime_t prev_irq_in_thread_total_ticks;

RUN_AFTER(WORKER_THREADS_INIT) {
    meas_begin_t = chVTGetSystemTimeX();
    idle_total_ticks = 0;
    irq_total_ticks = 0;
    irq_nesting = 0;
    idle_active = 0;
    irq_entered_from_idle = 0;
    irq_in_idle_total_ticks = 0;
    irq_in_thread_total_ticks = 0;
    prev_irq_in_idle_total_ticks = 0;
    prev_irq_in_thread_total_ticks = 0;
    worker_thread_add_timer_task(&WT, &load_print_task, load_print_task_func, NULL, chTimeMS2I(5000), true);
}

static void load_print_task_func(struct worker_thread_timer_task_s* task) {
    (void)task;

    systime_t tnow = chVTGetSystemTimeX();
    systime_t period_ticks = tnow - meas_begin_t;

    // Snapshot cumulative IRQ ticks (separate buckets) and compute deltas for this period
    systime_t irq_idle_snapshot = irq_in_idle_total_ticks;
    systime_t irq_thread_snapshot = irq_in_thread_total_ticks;
    systime_t period_irq_idle_ticks = irq_idle_snapshot - prev_irq_in_idle_total_ticks;
    systime_t period_irq_thread_ticks = irq_thread_snapshot - prev_irq_in_thread_total_ticks;
    prev_irq_in_idle_total_ticks = irq_idle_snapshot;
    prev_irq_in_thread_total_ticks = irq_thread_snapshot;

    // Calculate percentages (x100)
    uint32_t idle_pct_x100 = 0;
    uint32_t irq_pct_x100 = 0;
    uint32_t thread_pct_x100 = 0;
    if (period_ticks > 0) {
        // Remove IRQ time that occurred while the CPU was in the idle thread
        systime_t idle_effective_ticks = idle_total_ticks;
        if (period_irq_idle_ticks <= idle_effective_ticks) {
            idle_effective_ticks -= period_irq_idle_ticks;
        } else {
            idle_effective_ticks = 0;
        }

        idle_pct_x100 = (uint64_t)10000 * idle_effective_ticks / period_ticks;
        systime_t period_irq_total_ticks = period_irq_idle_ticks + period_irq_thread_ticks;
        irq_pct_x100 = (uint64_t)10000 * period_irq_total_ticks / period_ticks;
        uint32_t used_x100 = idle_pct_x100 + irq_pct_x100;
        thread_pct_x100 = (used_x100 <= 10000) ? (10000 - used_x100) : 0;
    }

    uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO, "load",
                          "idle %u.%02u%%, irq %u.%02u%%, thr %u.%02u%% (idle:%u irqI:%u irqT:%u total:%u)",
                          idle_pct_x100/100, idle_pct_x100%100,
                          irq_pct_x100/100, irq_pct_x100%100,
                          thread_pct_x100/100, thread_pct_x100%100,
                          idle_total_ticks, period_irq_idle_ticks, period_irq_thread_ticks, period_ticks);

    meas_begin_t = tnow;
    idle_total_ticks = 0;
}
