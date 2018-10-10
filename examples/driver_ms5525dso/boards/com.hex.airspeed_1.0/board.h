#pragma once

#include <stdint.h>
#include <modules/platform_stm32f302x8/platform_stm32f302x8.h>

#define BOARD_PAL_LINE_SPI_3_SCK PAL_LINE(GPIOB,3) // AF6
#define BOARD_PAL_LINE_SPI_3_MISO PAL_LINE(GPIOB,4) // AF6
#define BOARD_PAL_LINE_SPI_3_MOSI PAL_LINE(GPIOB,5) // AF6
#define BOARD_PAL_LINE_SPI_CS_MS5525DSO PAL_LINE(GPIOB,0)
#define BOARD_PAL_LINE_SPI_CS_ICM PAL_LINE(GPIOB,6)
#define BOARD_PAL_LINE_DEBUG_TX PAL_LINE(GPIOA,2)
#define BOARD_PAL_LINE_DEBUG_RX PAL_LINE(GPIOA,3)
#define BOARD_PAL_LINE_CAN_RX PAL_LINE(GPIOA,11) // AF9
#define BOARD_PAL_LINE_CAN_TX PAL_LINE(GPIOA,12) // AF9

#define MS5525DSO_SPI_BUS 3
#define ICM_SPI_BUS 3