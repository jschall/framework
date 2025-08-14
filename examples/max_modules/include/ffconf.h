/* FatFS config wrapper: reuse module-provided _ffconf.h and enable RT sync. */
#pragma once

#include "hal.h"
#include <ch.h>
/* Reuse the FatFS configuration shipped with the modules. */
#include "../../modules/uSD/fatfs/_ffconf.h"

/* For this example, disable FatFS reentrancy to avoid OS sync type deps. */
#undef FF_FS_REENTRANT
#define FF_FS_REENTRANT   0

