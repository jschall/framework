#include <hal.h>

void boardInit(void) {
    palSetLineMode(BOARD_PAL_LINE_CAN_RX, PAL_MODE_ALTERNATE(9) | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(BOARD_PAL_LINE_CAN_TX, PAL_MODE_ALTERNATE(9) | PAL_STM32_OSPEED_HIGHEST);
}

/* ChibiOS SDC hooks for F7 */
bool sdc_lld_is_card_inserted(SDCDriver *sdcp) {
    (void)sdcp;
    return true;
}

bool sdc_lld_is_write_protected(SDCDriver *sdcp) {
    (void)sdcp;
    return false;
}
