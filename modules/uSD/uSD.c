#include "uSD.h"
#include <common/ctor.h>
#include <hal.h>
#include <ch.h>

MEMORYPOOL_DECL(mutex_pool, sizeof(mutex_t), PORT_NATURAL_ALIGN, chCoreAllocAlignedI);

static FATFS filesystem;
static bool filesystem_ok;

static bool try_reformat(void) {
    FRESULT res;
    uint8_t workingbuf[1024];
    res = f_mkfs("/", FM_EXFAT, 0, workingbuf, sizeof(workingbuf));
    return res == FR_OK;
}

RUN_AFTER(CH_SYS_INIT) {
    sdcStart(&SDCD1, NULL);
    if (sdcConnect(&SDCD1) != HAL_SUCCESS) {
        return;
    }

    FRESULT res;

    for (uint8_t i=0; i<10; i++) {
        res = f_mount(&filesystem, "/", 1);
        if (res == FR_OK) {
            break;
        }
        chThdSleep(chTimeMS2I(10));
    }

#ifdef MICROSD_MOUNT_FAIL_REFORMAT
    if (res != FR_OK) {
        if (!try_reformat()) {
            return;
        }

        res = f_mount(&filesystem, "/", 1);
        if (res != FR_OK) {
            return;
        }
    }
#else
    (void)try_reformat; // silence unused-function if not used by feature
    if (res != FR_OK) {
        return;
    }
#endif

    filesystem_ok = true;
}

FATFS* uSD_get_filesystem(void) {
    if (!filesystem_ok) {
        return NULL;
    }

    return &filesystem;
}


// int ff_cre_syncobj (BYTE vol, FF_SYNC_t *sobj) {
//     *sobj = chPoolAlloc(&mutex_pool);
//     if (*sobj) {
//         chMtxObjectInit(*sobj);
//         return 1;
//     }
//     return 0;
// }
//
// int ff_del_syncobj(FF_SYNC_t sobj) {
//     chPoolFree(&mutex_pool, sobj);
//     return 1;
// }
//
// int ff_req_grant(FF_SYNC_t sobj) {
//     chMtxLock(sobj);
//     return 1;
// }
//
// void ff_rel_grant(FF_SYNC_t sobj)
// {
//     chMtxUnlock(sobj);
// }
