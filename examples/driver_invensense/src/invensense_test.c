#include <modules/driver_invensense/driver_invensense.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/uavcan_debug/uavcan_debug.h>

#define WT hpwork_thread
WORKER_THREAD_DECLARE_EXTERN(WT)

static struct invensense_instance_s invensense;

static struct worker_thread_timer_task_s invensense_test_task;
static void invensense_test_task_func(struct worker_thread_timer_task_s* task);

RUN_AFTER(INIT_END) {
    invensense_init(&invensense, 3, BOARD_PAL_LINE_SPI3_ICM_CS, INVENSENSE_IMU_TYPE_ICM20602, ACCEL_FS_16g, GYRO_FS_250dps, ODR_1KHz);
    worker_thread_add_timer_task(&WT, &invensense_test_task, invensense_test_task_func, NULL, MS2ST(1), true);
}

static void invensense_test_task_func(struct worker_thread_timer_task_s* task) {
    accel_data_t accel_data = invensense_get_accel(&invensense);
    uavcan_send_debug_keyvalue("AZ", accel_data.z);
}
