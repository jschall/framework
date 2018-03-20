#include <hal.h>

void boardInit(void) {
#ifndef TARGET_BOOTLOADER
    palSetLineMode(BOARD_PAL_LINE_SPI_SCK, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLDOWN); // SPI4 SCK
    palSetLineMode(BOARD_PAL_LINE_SPI_MISO, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST); // SPI4 MISO
    palSetLineMode(BOARD_PAL_LINE_SPI_MOSI, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST); // SPI4 MOSI
    palSetLineMode(BOARD_PAL_LINE_SPI_UWB_CS, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST); // UWB CS
    palSetLineMode(BOARD_PAL_LINE_UWB_IRQ, PAL_STM32_MODE_INPUT | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLDOWN); // UWB IRQ
#endif

    palSetLineMode(BOARD_PAL_LINE_CAN_RX, PAL_MODE_ALTERNATE(9) | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(BOARD_PAL_LINE_CAN_TX, PAL_MODE_ALTERNATE(9) | PAL_STM32_OSPEED_HIGHEST);
}
