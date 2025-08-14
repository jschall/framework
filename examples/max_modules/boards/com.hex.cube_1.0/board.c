#include <hal.h>

void boardInit(void) {
    palSetLineMode(BOARD_PAL_LINE_CAN_RX, PAL_MODE_ALTERNATE(9) | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(BOARD_PAL_LINE_CAN_TX, PAL_MODE_ALTERNATE(9) | PAL_STM32_OSPEED_HIGHEST);
    /* SPI1 default pins on Cube F4 (PA5=SCK, PA6=MISO, PA7=MOSI) */
    palSetLineMode(PAL_LINE(GPIOA,5), PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(PAL_LINE(GPIOA,6), PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(PAL_LINE(GPIOA,7), PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
}

/* ChibiOS SDC hooks: provide simple stubs so the SDC driver links. */
bool sdc_lld_is_card_inserted(SDCDriver *sdcp) {
    (void)sdcp;
    return true;
}

bool sdc_lld_is_write_protected(SDCDriver *sdcp) {
    (void)sdcp;
    return false;
}


