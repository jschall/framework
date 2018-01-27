void __attribute__((weak)) __early_init_hook(void) {}
void __attribute__((weak)) board_clock_init(void) {}
void __attribute__((weak)) boot_msg_retrieve(void) {}

void __early_init(void) {
    boot_msg_retrieve();

    __early_init_hook();

    board_clock_init();
}
