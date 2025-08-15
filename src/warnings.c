#include <ch.h>

#if !CH_DBG_SYSTEM_STATE_CHECK
#pragma message("Consider enabling CH_DBG_SYSTEM_STATE_CHECK in framework_conf.h.")
#endif

#if !CH_DBG_ENABLE_CHECKS
#pragma message("Consider enabling CH_DBG_ENABLE_CHECKS in framework_conf.h.")
#endif

#if !CH_DBG_ENABLE_ASSERTS
#pragma message("Consider enabling CH_DBG_ENABLE_ASSERTS in framework_conf.h.")
#endif

#if !CH_DBG_ENABLE_STACK_CHECK
#pragma message("Consider enabling CH_DBG_ENABLE_STACK_CHECK in framework_conf.h.")
#endif

#ifndef MODULE_FREEMEM_CHECK_ENABLED
#pragma message("Consider enabling module freemem_check.")
#endif
