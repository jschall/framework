#pragma once

#if !defined(_FROM_ASM_)
#include <stdint.h>
#endif
#include <modules/platform_stm32f427/platform_stm32f427.h>

#define BOARD_PAL_LINE_CAN_RX PAL_LINE(GPIOD,0)
#define BOARD_PAL_LINE_CAN_TX PAL_LINE(GPIOD,1)

/* SPI lines for ICM on Cube (F4) */
#define BOARD_PAL_LINE_SPI3_SCK  PAL_LINE(GPIOC,10)
#define BOARD_PAL_LINE_SPI3_MISO PAL_LINE(GPIOC,11)
#define BOARD_PAL_LINE_SPI3_MOSI PAL_LINE(GPIOC,12)
#define BOARD_PAL_LINE_SPI3_ICM_CS PAL_LINE(GPIOA,15)
