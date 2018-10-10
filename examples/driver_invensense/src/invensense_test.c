#include <modules/driver_invensense/driver_invensense.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/uavcan_debug/uavcan_debug.h>

#include <uavcan.equipment.ahrs.RawIMU.h>

#include <math.h>
#include <hal.h>
#include <string.h>

#define WT hpwork_thread
WORKER_THREAD_DECLARE_EXTERN(WT)
WORKER_THREAD_DECLARE_EXTERN(lpwork_thread)
WORKER_THREAD_DECLARE_EXTERN(mpwork_thread)

// static UARTConfig usart2conf = { NULL, NULL, NULL, NULL, NULL, NULL, 0, /* NOTE: actually 8000000 */ 4500000, USART_CR1_OVER8, 0, 0 };

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

static systime_t prev_timestamp;
static float dt = 1/32000.0;
static float dt_sum;
static float x[2][7];
static uint8_t x_idx;
static uint32_t processed_count;

RUN_AFTER(INIT_END) {
//     uartStart(&UARTD2, &usart2conf);

    chMtxObjectInit(&buf[0].mtx);
    chMtxObjectInit(&buf[1].mtx);

    invensense_init(&invensense, 3, BOARD_PAL_LINE_SPI_CS_ICM, INVENSENSE_IMU_TYPE_ICM20602);
    worker_thread_add_timer_task(&WT, &invensense_test_task, invensense_test_task_func, NULL, LL_MS2ST(1), true);
    worker_thread_add_timer_task(&lpwork_thread, &report_task, report_task_func, NULL, LL_S2ST(1), true);
    worker_thread_add_timer_task(&mpwork_thread, &data_processor_task, data_processor_task_func, NULL, TIME_INFINITE, false);

    memset(x, 0, sizeof(x));
    x[0][0] = 1;
}

static void invensense_test_task_func(struct worker_thread_timer_task_s* task) {
    buf_idx = (buf_idx+1)%2;
    chMtxLock(&buf[buf_idx].mtx);
    buf[buf_idx].timestamp = chVTGetSystemTimeX();
    buf[buf_idx].count = invensense_read_fifo(&invensense, buf[buf_idx].data)/14;
    meas_count += buf[buf_idx].count;
    chMtxUnlock(&buf[buf_idx].mtx);
    worker_thread_timer_task_reschedule(&mpwork_thread, &data_processor_task, TIME_IMMEDIATE);
}

static void integrate(float* x, float* omega, float* accel, float dt, float* x_ret);

static void data_processor_task_func(struct worker_thread_timer_task_s* task) {
    uint8_t buf_idx_tmp = buf_idx;
    chMtxLock(&buf[buf_idx_tmp].mtx);

    float dt_estimated = (float)(buf[buf_idx_tmp].timestamp - prev_timestamp) / buf[buf_idx_tmp].count / LL_S2ST(1);

    prev_timestamp = buf[buf_idx_tmp].timestamp;

    if (dt_estimated < 1.5/32000.0 && dt_estimated > 0.5/32000.0) {
        dt += (dt_estimated-dt)*0.001;
    }

    for (uint8_t i=0; i<buf[buf_idx_tmp].count; i++) {
        uint8_t prev_x_idx = x_idx;
        x_idx = (x_idx+1)%2;

        float omega[] = { buf[buf_idx_tmp].data[i].gyro_x*0.0010652969463144809, buf[buf_idx_tmp].data[i].gyro_y*0.0010652969463144809, buf[buf_idx_tmp].data[i].gyro_z*0.0010652969463144809 };
        float accel[] = { buf[buf_idx_tmp].data[i].accel_x*0.004788500625629444, buf[buf_idx_tmp].data[i].accel_y*0.004788500625629444, buf[buf_idx_tmp].data[i].accel_z*0.004788500625629444 };

        integrate(x[prev_x_idx], omega, accel, dt, x[x_idx]);
        dt_sum += dt;
        processed_count++;

        if (dt_sum >= (1.0/100.0)) {
            struct uavcan_equipment_ahrs_RawIMU_s msg;
            memset(&msg, 0, sizeof(msg));
            msg.integration_interval = dt_sum;

            msg.rate_gyro_latest[0] = buf[buf_idx_tmp].data[buf[buf_idx_tmp].count-1].gyro_x*0.0010652969463144809;
            msg.rate_gyro_latest[1] = buf[buf_idx_tmp].data[buf[buf_idx_tmp].count-1].gyro_y*0.0010652969463144809;
            msg.rate_gyro_latest[2] = buf[buf_idx_tmp].data[buf[buf_idx_tmp].count-1].gyro_z*0.0010652969463144809;

            msg.accelerometer_latest[0] = buf[buf_idx_tmp].data[buf[buf_idx_tmp].count-1].accel_x*0.004788500625629444;
            msg.accelerometer_latest[1] = buf[buf_idx_tmp].data[buf[buf_idx_tmp].count-1].accel_y*0.004788500625629444;
            msg.accelerometer_latest[2] = buf[buf_idx_tmp].data[buf[buf_idx_tmp].count-1].accel_z*0.004788500625629444;

            float l = sqrtf(((x[x_idx][1])*(x[x_idx][1])) + ((x[x_idx][2])*(x[x_idx][2])) + ((x[x_idx][3])*(x[x_idx][3])));

            if(l == 0) {
                msg.rate_gyro_integral[0] = 0;
                msg.rate_gyro_integral[1] = 0;
                msg.rate_gyro_integral[2] = 0;
            } else {
                msg.rate_gyro_integral[0] = 2.0*atan2f(l,x[x_idx][0])/l * x[x_idx][1];
                msg.rate_gyro_integral[1] = 2.0*atan2f(l,x[x_idx][0])/l * x[x_idx][2];
                msg.rate_gyro_integral[2] = 2.0*atan2f(l,x[x_idx][0])/l * x[x_idx][3];
            }

            msg.accelerometer_integral[0] = x[x_idx][4];
            msg.accelerometer_integral[1] = x[x_idx][5];
            msg.accelerometer_integral[2] = x[x_idx][6];

            uavcan_broadcast(0, &uavcan_equipment_ahrs_RawIMU_descriptor, CANARD_TRANSFER_PRIORITY_HIGH, &msg);

            dt_sum = 0;
            memset(x, 0, sizeof(x));
            x[x_idx][0] = 1;
        }
    }

    chMtxUnlock(&buf[buf_idx_tmp].mtx);
}

static void report_task_func(struct worker_thread_timer_task_s* task) {
    chSysLock();
    uint32_t meas_count_tmp = meas_count;
    meas_count = 0;
    uint32_t processed_count_tmp = processed_count;
    processed_count = 0;
    chSysUnlock();

//     uavcan_send_debug_keyvalue("cnt", (float)meas_count_tmp);
//     uavcan_send_debug_keyvalue("proc", (float)processed_count_tmp);
//     uavcan_send_debug_keyvalue("stat", (float)invensense_read_int_status(&invensense));
}

static void integrate(float* x, float* omega, float* accel, float dt, float* x_ret) {
    float X0 = (1.0f/2.0f)*omega[0];
    float X1 = (1.0f/2.0f)*omega[1];
    float X2 = (1.0f/2.0f)*omega[2];
    float X3 = dt*(-X0*x[1] - X1*x[2] - X2*x[3]) + x[0];
    float X4 = dt*(X0*x[0] - X1*x[3] + X2*x[2]) + x[1];
    float X5 = (1.0f/2.0f)*x[0];
    float X6 = dt*(X0*x[3] - X2*x[1] + X5*omega[1]) + x[2];
    float X7 = dt*(-X0*x[2] + X1*x[1] + X5*omega[2]) + x[3];
    float X8 = 1/(sqrtf(((fabsf(X3))*(fabsf(X3))) + ((fabsf(X4))*(fabsf(X4))) + ((fabsf(X6))*(fabsf(X6))) + ((fabsf(X7))*(fabsf(X7)))));
    float X9 = ((x[0])*(x[0]));
    float X10 = ((x[2])*(x[2]));
    float X11 = -X10;
    float X12 = ((x[3])*(x[3]));
    float X13 = -X12;
    float X14 = ((x[1])*(x[1]));
    float X15 = 2.0*x[1];
    float X16 = X15*x[2];
    float X17 = 2.0*x[0];
    float X18 = X17*x[3];
    float X19 = X17*x[2];
    float X20 = X15*x[3];
    float X21 = -X14 + X9;
    float X22 = 2.0*x[2]*x[3];
    float X23 = X15*x[0];

    x_ret[0] = X3*X8;
    x_ret[1] = X4*X8;
    x_ret[2] = X6*X8;
    x_ret[3] = X7*X8;
    x_ret[4] = dt*(accel[0]*(X11 + X13 + 1.0*X14 + X9) + accel[1]*(X16 - X18) + accel[2]*(X19 + X20)) + x[4];
    x_ret[5] = dt*(accel[0]*(X16 + X18) + accel[1]*(1.0*X10 + X13 + X21) + accel[2]*(X22 - X23)) + x[5];
    x_ret[6] = dt*(accel[0]*(-X19 + X20) + accel[1]*(X22 + X23) + accel[2]*(X11 + 1.0*X12 + X21)) + x[6];
}
