#pragma once

#if !defined(_FROM_ASM_)
#include <stdint.h>
#endif
#include <modules/platform_stm32f767/platform_stm32f767.h>

#define BOARD_CONFIG_HW_NAME "com.hex.cube"
#define BOARD_CONFIG_HW_MAJOR_VER 2
#define BOARD_CONFIG_HW_MINOR_VER 0

#define BOARD_CONFIG_HW_INFO_STRUCTURE { \
    .hw_name = BOARD_CONFIG_HW_NAME, \
    .hw_major_version = BOARD_CONFIG_HW_MAJOR_VER, \
    .hw_minor_version = BOARD_CONFIG_HW_MINOR_VER, \
    .board_desc_fmt = SHARED_HW_INFO_BOARD_DESC_FMT_NONE, \
    .board_desc = 0, \
}

#define BOARD_PAL_LINE_CAN_RX PAL_LINE(GPIOD,0)
#define BOARD_PAL_LINE_CAN_TX PAL_LINE(GPIOD,1)

/* Required by STM32F7 hal_lld */
#define STM32_LSECLK 0U
#define STM32_LSEDRV (3U)

/* Flash geometry for STM32F767 (2 MB) required by flash driver */
#define BOARD_FLASH_SIZE 2048
