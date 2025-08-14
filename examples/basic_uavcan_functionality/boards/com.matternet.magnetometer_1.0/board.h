#pragma once

#if !defined(_FROM_ASM_)
#include <stdint.h>
#endif
#include <modules/platform_stm32f302x8/platform_stm32f302x8.h>

#define BOARD_CONFIG_HW_NAME "com.matternet.magnetometer"
#define BOARD_CONFIG_HW_MAJOR_VER 1
#define BOARD_CONFIG_HW_MINOR_VER 0

#define BOARD_CONFIG_HW_INFO_STRUCTURE { \
    .hw_name = BOARD_CONFIG_HW_NAME, \
    .hw_major_version = BOARD_CONFIG_HW_MAJOR_VER, \
    .hw_minor_version = BOARD_CONFIG_HW_MINOR_VER, \
    .board_desc_fmt = SHARED_HW_INFO_BOARD_DESC_FMT_NONE, \
    .board_desc = 0, \
}



#define BOARD_PAL_LINE_SPI3_SCK PAL_LINE(GPIOB,3)
#define BOARD_PAL_LINE_SPI3_MISO PAL_LINE(GPIOB,4)
#define BOARD_PAL_LINE_SPI3_MOSI PAL_LINE(GPIOB,5)
#define BOARD_PAL_LINE_SPI3_ICM_CS PAL_LINE(GPIOB,6)
#define BOARD_PAL_LINE_SPI3_ICM_INT PAL_LINE(GPIOB,7)

#define BOARD_PAL_LINE_I2C_SLAVE_SCL PAL_LINE(GPIOA,9)
#define BOARD_PAL_LINE_I2C_SLAVE_SDA PAL_LINE(GPIOA,10)

#define BOARD_PAL_LINE_CAN_RX PAL_LINE(GPIOA,11)
#define BOARD_PAL_LINE_CAN_TX PAL_LINE(GPIOA,12)
