#include "uSD.h"
#include <common/ctor.h>
#include <hal.h>
#include <ch.h>
#include <faults.h>
#include <stdarg.h>
#include <chprintf.h>
#include <modules/uavcan_debug/uavcan_debug.h>
#include <modules/worker_thread/worker_thread.h>
#include <ChibiOS/os/hal/ports/STM32/LLD/SDMMCv2/hal_sdc_lld.h>

MEMORYPOOL_DECL(mutex_pool, sizeof(mutex_t), PORT_NATURAL_ALIGN, chCoreAllocAlignedI);

static FATFS filesystem;
static bool filesystem_ok;

/* Debug helpers. Keep short messages to avoid flooding logs. */
static void usd_debug_logf(const char *fmt, ...) {
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    (void)chvsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    uavcan_send_debug_msg(LOG_LEVEL_INFO, "uSD", "%s", buf);
}

static void usd_dump_regs(SDCDriver *sdcp, const char *phase) {
    SDMMC_TypeDef *s = sdcp->sdmmc;
    uint32_t clkcr = s->CLKCR;
    uint32_t div_field = (clkcr & SDMMC_CLKCR_CLKDIV_Msk);
    uint32_t fker = sdcp->clkfreq;
    /* Approximate sdclk using the divider field as used by this driver. */
    uint32_t sdclk = (div_field != 0U) ? (fker / (2U * div_field)) : 0U;

    usd_debug_logf("%s: CD=%d WP=%d fker=%lu sdclk~=%lu DIV=%lu",
                   phase,
                   sdc_lld_is_card_inserted(sdcp) ? 1 : 0,
                   sdc_lld_is_write_protected(sdcp) ? 1 : 0,
                   (unsigned long)fker,
                   (unsigned long)sdclk,
                   (unsigned long)div_field);

    usd_debug_logf("%s: POWER=0x%08lX CLKCR=0x%08lX STA=0x%08lX MASK=0x%08lX",
                   phase,
                   (unsigned long)s->POWER,
                   (unsigned long)clkcr,
                   (unsigned long)s->STA,
                   (unsigned long)s->MASK);

    usd_debug_logf("%s: DTIMER=%lu DCTRL=0x%08lX DLEN=%lu",
                   phase,
                   (unsigned long)s->DTIMER,
                   (unsigned long)s->DCTRL,
                   (unsigned long)s->DLEN);
}

static bool try_reformat(void) {
    FRESULT res;
    uint8_t workingbuf[1024];
    res = f_mkfs("/", FM_EXFAT, 0, workingbuf, sizeof(workingbuf));
    return res == FR_OK;
}

// Configure default severity if not provided in framework_conf.h
#ifndef USD_FAULT_SEVERITY
#define USD_FAULT_SEVERITY UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR
#endif

static struct worker_thread_timer_task_s usd_init_task;

static void usd_init_task_func(struct worker_thread_timer_task_s* task);

/* Use the configured timing worker thread for scheduling. */
#ifndef USD_WORKER_THREAD
#define USD_WORKER_THREAD TIMING_WORKER_THREAD
#endif
#define WT USD_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

RUN_AFTER(INIT_END) {
    /* Raise fault if no card detected at startup; cleared on successful mount. */
    if (!sdc_lld_is_card_inserted(&SDCD1)) {
        fault_set("uSDfault", USD_FAULT_SEVERITY, "no card");
    }
    /* Periodic 100ms task to (re)try SD initialization. */
    worker_thread_add_timer_task(&WT, &usd_init_task, usd_init_task_func, NULL, chTimeMS2I(100), true);
}

static void usd_init_task_func(struct worker_thread_timer_task_s* task) {
    (void)task;
    static bool init_in_progress;
    static systime_t next_attempt = 0;

    if (filesystem_ok) {
        /* Stop scheduling once mounted. */
        worker_thread_remove_timer_task(&WT, &usd_init_task);
        return;
    }

    systime_t now = osalOsGetSystemTimeX();
    if (init_in_progress) {
        return;
    }
    if (!osalTimeIsInRangeX(now, next_attempt, osalTimeAddX(next_attempt, TIME_IMMEDIATE))) {
        return;
    }

    init_in_progress = true;

    /* If no card is inserted, set fault and back off without touching the driver. */
    if (!sdc_lld_is_card_inserted(&SDCD1)) {
        fault_set("uSDfault", USD_FAULT_SEVERITY, "no card");
        next_attempt = osalTimeAddX(now, OSAL_MS2I(500));
        init_in_progress = false;
        return;
    }

    sdcStart(&SDCD1, NULL);
    usd_dump_regs(&SDCD1, "after sdcStart");
    if (sdcConnect(&SDCD1) != HAL_SUCCESS) {
        sdcflags_t errs = sdcGetAndClearErrors(&SDCD1);
        usd_dump_regs(&SDCD1, "after sdcConnect FAIL");
        usd_debug_logf("errors=0x%08lX", (unsigned long)errs);
        fault_set("uSDfault", USD_FAULT_SEVERITY, "connect fail");
        sdcStop(&SDCD1);
        next_attempt = osalTimeAddX(now, OSAL_MS2I(500));
        init_in_progress = false;
        return;
    }
    usd_dump_regs(&SDCD1, "after sdcConnect OK");

    FRESULT res;

    for (uint8_t i = 0; i < 10; i++) {
        res = f_mount(&filesystem, "/", 1);
        if (res == FR_OK) {
            break;
        }
        chThdSleep(chTimeMS2I(10));
    }

    if (res != FR_OK) {
#ifdef MICROSD_MOUNT_FAIL_REFORMAT
        if (!try_reformat()) {
            fault_set("uSDfault", USD_FAULT_SEVERITY, "mount fail");
            sdcDisconnect(&SDCD1);
            sdcStop(&SDCD1);
            next_attempt = osalTimeAddX(now, OSAL_MS2I(500));
            init_in_progress = false;
            return;
        }
        res = f_mount(&filesystem, "/", 1);
        if (res != FR_OK) {
            fault_set("uSDfault", USD_FAULT_SEVERITY, "mount fail");
            sdcDisconnect(&SDCD1);
            sdcStop(&SDCD1);
            next_attempt = osalTimeAddX(now, OSAL_MS2I(500));
            init_in_progress = false;
            return;
        }
#else
        (void)try_reformat;
        fault_set("uSDfault", USD_FAULT_SEVERITY, "mount fail");
        sdcDisconnect(&SDCD1);
        sdcStop(&SDCD1);
        next_attempt = osalTimeAddX(now, OSAL_MS2I(500));
        init_in_progress = false;
        return;
#endif
    }

    filesystem_ok = true;
    fault_clear("uSDfault");
    /* basic info */
    DWORD free_clust = 0;
    FATFS* fs;
    if (f_getfree("/", &free_clust, &fs) == FR_OK && fs) {
        uint32_t sector_size =
#if FF_MAX_SS != FF_MIN_SS
            fs->ssize;
#else
            512;
#endif
        uint64_t free_bytes = (uint64_t)free_clust * (uint64_t)fs->csize * (uint64_t)sector_size;
        usd_debug_logf("mounted: free=%lu KB", (unsigned long)(free_bytes / 1024U));
    } else {
        usd_debug_logf("mounted");
    }
    /* Stop periodic attempts after success. */
    worker_thread_remove_timer_task(&WT, &usd_init_task);
    init_in_progress = false;
}

FATFS* uSD_get_filesystem(void) {
    if (!filesystem_ok) {
        return NULL;
    }

    return &filesystem;
}
