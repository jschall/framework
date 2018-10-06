#include <modules/driver_invensense/driver_invensense.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/uavcan_debug/uavcan_debug.h>

#include <hal.h>
#include <string.h>

#define WT hpwork_thread
WORKER_THREAD_DECLARE_EXTERN(WT)
WORKER_THREAD_DECLARE_EXTERN(lpwork_thread)

static UARTConfig usart2conf = { NULL, NULL, NULL, NULL, NULL, NULL, 0, /* NOTE: actually 8000000 */ 4500000, USART_CR1_OVER8, 0, 0 };

static struct invensense_instance_s invensense;

static struct worker_thread_timer_task_s invensense_test_task;
static void invensense_test_task_func(struct worker_thread_timer_task_s* task);

static struct worker_thread_timer_task_s data_processor_task;
static void data_processor_task_func(struct worker_thread_timer_task_s* task);

static struct worker_thread_timer_task_s report_task;
static void report_task_func(struct worker_thread_timer_task_s* task);

static uint32_t meas_count;

static struct {
    struct {
        int16_t accel_x;
        int16_t accel_y;
        int16_t accel_z;
        int16_t temp;
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
    } data[72];
    systime_t timestamp;
    uint8_t count;
    mutex_t mtx;
} buf[2];
static uint8_t buf_idx;

RUN_AFTER(INIT_END) {
    uartStart(&UARTD2, &usart2conf);

    chMtxObjectInit(&buf[0].mtx);
    chMtxObjectInit(&buf[1].mtx);

    invensense_init(&invensense, 3, BOARD_PAL_LINE_SPI_CS_ICM, INVENSENSE_IMU_TYPE_ICM20602);
    worker_thread_add_timer_task(&WT, &invensense_test_task, invensense_test_task_func, NULL, LL_MS2ST(1), true);
    worker_thread_add_timer_task(&lpwork_thread, &report_task, report_task_func, NULL, LL_S2ST(1), true);
    worker_thread_add_timer_task(&lpwork_thread, &data_processor_task, data_processor_task_func, NULL, TIME_INFINITE, false);

}

static void invensense_test_task_func(struct worker_thread_timer_task_s* task) {
    buf_idx = (buf_idx+1)%2;
    chMtxLock(&buf[buf_idx].mtx);
    buf[buf_idx].timestamp = chVTGetSystemTimeX();
    buf[buf_idx].count = invensense_read_fifo(&invensense, buf[buf_idx].data)/14;
    meas_count += buf[buf_idx].count;
    chMtxUnlock(&buf[buf_idx].mtx);
    worker_thread_timer_task_reschedule(&lpwork_thread, &data_processor_task, TIME_IMMEDIATE);
}

static void data_processor_task_func(struct worker_thread_timer_task_s* task) {
    uint8_t buf_idx_tmp = buf_idx;
    chMtxLock(&buf[buf_idx_tmp].mtx);

    systime_t tstart = chVTGetSystemTimeX();
    for (uint8_t i=0; i<buf[buf_idx].count; i++) {
        size_t len = sizeof(buf[buf_idx].data[i]);
        uartSendTimeout(&UARTD2, &len, &buf[buf_idx].data[i], TIME_INFINITE);
    }
//     uavcan_send_debug_keyvalue("t", (float)(chVTGetSystemTimeX()-tstart));

    chMtxUnlock(&buf[buf_idx_tmp].mtx);
}

static void report_task_func(struct worker_thread_timer_task_s* task) {
    chSysLock();
    uint32_t meas_count_tmp = meas_count;
    meas_count = 0;
    chSysUnlock();

    uavcan_send_debug_keyvalue("cnt", (float)meas_count_tmp);
    uavcan_send_debug_keyvalue("stat", (float)invensense_read_int_status(&invensense));
//     uartStartSend(&UARTD2, strlen("hello world\n"), "hello world\n");
}
