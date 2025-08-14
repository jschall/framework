#pragma once

#if !defined(_FROM_ASM_)
#if !defined(_FROM_ASM_)
#include <stdint.h>
#endif
#include <modules/platform_stm32f427/platform_stm32f427.h>
#endif

#define BOARD_CONFIG_HW_NAME "com.hex.cube"
#define BOARD_CONFIG_HW_MAJOR_VER 1
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

/* SPI3 lines for F427 Cube */
#define BOARD_PAL_LINE_SPI3_SCK  PAL_LINE(GPIOC,10)
#define BOARD_PAL_LINE_SPI3_MISO PAL_LINE(GPIOC,11)
#define BOARD_PAL_LINE_SPI3_MOSI PAL_LINE(GPIOC,12)
#define BOARD_PAL_LINE_SPI3_ICM_CS PAL_LINE(GPIOA,15)

