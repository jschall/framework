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

static float dt_sum;
static float _x[2][7];
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

    memset(_x, 0, sizeof(_x));
    _x[0][0] = 1;
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

    static const float dt = 1/32000.0;

    for (uint8_t i=0; i<buf[buf_idx_tmp].count; i++) {
        uint8_t prev_x_idx = x_idx;
        x_idx = (x_idx+1)%2;

        float omega[] = { buf[buf_idx_tmp].data[i].gyro_x*0.0010652969463144809, buf[buf_idx_tmp].data[i].gyro_y*0.0010652969463144809, buf[buf_idx_tmp].data[i].gyro_z*0.0010652969463144809 };
        float accel[] = { buf[buf_idx_tmp].data[i].accel_x*0.004788500625629444, buf[buf_idx_tmp].data[i].accel_y*0.004788500625629444, buf[buf_idx_tmp].data[i].accel_z*0.004788500625629444 };

        integrate(_x[prev_x_idx], omega, accel, dt, _x[x_idx]);
        dt_sum += dt;
        processed_count++;
    }

    if (dt_sum >= 1) {
        struct uavcan_equipment_ahrs_RawIMU_s msg;
        memset(&msg, 0, sizeof(msg));
        msg.integration_interval = dt_sum;

        msg.rate_gyro_latest[0] = buf[buf_idx_tmp].data[buf[buf_idx_tmp].count-1].gyro_x*0.0010652969463144809;
        msg.rate_gyro_latest[1] = buf[buf_idx_tmp].data[buf[buf_idx_tmp].count-1].gyro_y*0.0010652969463144809;
        msg.rate_gyro_latest[2] = buf[buf_idx_tmp].data[buf[buf_idx_tmp].count-1].gyro_z*0.0010652969463144809;

        msg.accelerometer_latest[0] = buf[buf_idx_tmp].data[buf[buf_idx_tmp].count-1].accel_x*0.004788500625629444;
        msg.accelerometer_latest[1] = buf[buf_idx_tmp].data[buf[buf_idx_tmp].count-1].accel_y*0.004788500625629444;
        msg.accelerometer_latest[2] = buf[buf_idx_tmp].data[buf[buf_idx_tmp].count-1].accel_z*0.004788500625629444;

        float l = sqrtf(((_x[x_idx][1])*(_x[x_idx][1])) + ((_x[x_idx][2])*(_x[x_idx][2])) + ((_x[x_idx][3])*(_x[x_idx][3])));

        msg.rate_gyro_integral[0] = 2.0*atan2f(l,_x[x_idx][0])/l * _x[x_idx][1];
        msg.rate_gyro_integral[1] = 2.0*atan2f(l,_x[x_idx][0])/l * _x[x_idx][2];
        msg.rate_gyro_integral[2] = 2.0*atan2f(l,_x[x_idx][0])/l * _x[x_idx][3];

        msg.accelerometer_integral[0] = _x[x_idx][4];
        msg.accelerometer_integral[1] = _x[x_idx][5];
        msg.accelerometer_integral[2] = _x[x_idx][6];

        uavcan_broadcast(0, &uavcan_equipment_ahrs_RawIMU_descriptor, CANARD_TRANSFER_PRIORITY_HIGH, &msg);

        dt_sum = 0;
        memset(_x, 0, sizeof(_x));
        _x[x_idx][0] = 1;
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

    uavcan_send_debug_keyvalue("cnt", (float)meas_count_tmp);
    uavcan_send_debug_keyvalue("proc", (float)processed_count_tmp);
    uavcan_send_debug_keyvalue("stat", (float)invensense_read_int_status(&invensense));
}

static void integrate(float* x, float* omega, float* accel, float dt, float* x_ret) {
    float X0 = (1.0f/2.0f)*omega[0];
    float X1 = (1.0f/2.0f)*omega[2];
    float X2 = (1.0f/2.0f)*omega[1];
    float X3 = X0*x[0] + X1*x[2] - X2*x[3];
    float X4 = (1.0f/3.0f)*dt;
    float X5 = -X3*X4 + x[1];
    float X6 = dt*(X0*x[3] - X1*x[1] + X2*x[0]);
    float X7 = -1.0f/3.0f*X6 + x[2];
    float X8 = -X0*x[2] + X1*x[0] + X2*x[1];
    float X9 = -X4*X8 + x[3];
    float X10 = (3.0f/8.0f)*dt;
    float X11 = -X0*x[1] - X1*x[3] - X2*x[2];
    float X12 = (5.0f/8.0f)*dt;
    float X13 = -X11*X4 + x[0];
    float X14 = ((X13)*(X13));
    float X15 = ((X7)*(X7));
    float X16 = -X15;
    float X17 = ((X9)*(X9));
    float X18 = -X17;
    float X19 = ((X5)*(X5));
    float X20 = 2.0*x[1];
    float X21 = X20 - 0.666666666666667*X3*dt;
    float X22 = X21*X7;
    float X23 = 2.0*x[0];
    float X24 = -0.666666666666667*X11*dt + X23;
    float X25 = X24*X9;
    float X26 = X24*X7;
    float X27 = X21*X9;
    float X28 = ((x[0])*(x[0]));
    float X29 = ((x[2])*(x[2]));
    float X30 = -X29;
    float X31 = ((x[3])*(x[3]));
    float X32 = -X31;
    float X33 = ((x[1])*(x[1]));
    float X34 = X20*x[2];
    float X35 = X23*x[3];
    float X36 = X23*x[2];
    float X37 = X20*x[3];
    float X38 = X14 - X19;
    float X39 = 2.0*X7*X9;
    float X40 = X13*X21;
    float X41 = X28 - X33;
    float X42 = 2.0*x[2]*x[3];
    float X43 = X20*x[0];

    x_ret[0] = X10*(-X0*X5 - X1*X9 - X2*X7) + X11*X12 + x[0];
    x_ret[1] = X10*(X0*X13 + X1*X7 - X2*X9) + X12*X3 + x[1];
    x_ret[2] = X10*(X0*X9 - X1*X5 + X13*X2) + (5.0f/8.0f)*X6 + x[2];
    x_ret[3] = X10*(-X0*X7 + X1*X13 + X2*X5) + X12*X8 + x[3];
    x_ret[4] = X10*(accel[0]*(X14 + X16 + X18 + 1.0*X19) + accel[1]*(X22 - X25) + accel[2]*(X26 + X27)) + X12*(accel[0]*(X28 + X30 + X32 + 1.0*X33) + accel[1]*(X34 - X35) + accel[2]*(X36 + X37)) + x[4];
    x_ret[5] = X10*(accel[0]*(X22 + X25) + accel[1]*(1.0*X15 + X18 + X38) + accel[2]*(X39 - X40)) + X12*(accel[0]*(X34 + X35) + accel[1]*(1.0*X29 + X32 + X41) + accel[2]*(X42 - X43)) + x[5];
    x_ret[6] = X10*(accel[0]*(-X26 + X27) + accel[1]*(X39 + X40) + accel[2]*(X16 + 1.0*X17 + X38)) + X12*(accel[0]*(-X36 + X37) + accel[1]*(X42 + X43) + accel[2]*(X30 + 1.0*X31 + X41)) + x[6];
}
