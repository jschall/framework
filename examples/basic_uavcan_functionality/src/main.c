#include <common/ctor.h>
#include <modules/uavcan/uavcan.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/timing/timing.h>
#include <modules/uavcan_debug/uavcan_debug.h>

#if __GNUC__ != 6 || __GNUC_MINOR__ != 3 || __GNUC_PATCHLEVEL__ != 1
#error Please use arm-none-eabi-gcc 6.3.1.
#endif

#include <hal.h>

WORKER_THREAD_SPAWN(publish_thread, HIGHPRIO, 1024)

static struct worker_thread_timer_task_s my_task;

static void my_task_func(struct worker_thread_timer_task_s* task);

static void uart_char_recv(UARTDriver* uartp, uint16_t c);

static UARTConfig uart1conf = {.speed=9600, .rxchar_cb=uart_char_recv};

static uint32_t char_recv_ms;
static char scanned_code[255];
static size_t scanned_code_len;

RUN_BEFORE(INIT_END) {
    uartStart(&UARTD1, &uart1conf);
    worker_thread_add_timer_task(&publish_thread, &my_task, my_task_func, NULL, LL_MS2ST(10), true);
}

static void my_task_func(struct worker_thread_timer_task_s* task) {
    (void)task;

    chSysLock();
    if (millis()-char_recv_ms > 100 && scanned_code_len > 0) {
        char scanned_code_copy[256];
        memcpy(scanned_code_copy, scanned_code, scanned_code_len);
        scanned_code_copy[scanned_code_len] = '\0';
        scanned_code_len = 0;
        chSysUnlock();

        uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", scanned_code_copy);

        palSetLine(BOARD_PAL_LINE_LED2);
        chThdSleepMilliseconds(20);
        palClearLine(BOARD_PAL_LINE_LED2);
    } else {
        chSysUnlock();
    }
}

static void uart_char_recv(UARTDriver* uartp, uint16_t c) {
    if (scanned_code_len < 255) {
        char_recv_ms = millis();
        scanned_code[scanned_code_len++] = c;
        palToggleLine(BOARD_PAL_LINE_LED1);
    }
}
