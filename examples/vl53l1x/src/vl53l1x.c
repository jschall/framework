#include <modules/worker_thread/worker_thread.h>
#include <hal.h>

#include <modules/uavcan_debug/uavcan_debug.h>

#include <modules/driver_vl53l1x/vl53l1_api.h>

#define WT hpwork_thread
WORKER_THREAD_DECLARE_EXTERN(WT)

static const I2CConfig i2cconfig = {
  STM32_TIMINGR_PRESC(11U) |
  STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(3U) |
  STM32_TIMINGR_SCLH(3U)  | STM32_TIMINGR_SCLL(9U),
  0,
  0
};

static VL53L1_Dev_t vl53l1x;

static struct worker_thread_timer_task_s vl53l1x_test_task;
static void vl53l1x_test_task_func(struct worker_thread_timer_task_s* task);


RUN_AFTER(INIT_END) {
    i2cStart(&I2CD1, &i2cconfig);
    
    vl53l1x.bus = &I2CD1;
    vl53l1x.i2c_address = 0x29;
    
    VL53L1_Error status;
    
    status = VL53L1_WaitDeviceBooted(&vl53l1x);
//     uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "%u %u", __LINE__, status);
    status = VL53L1_DataInit(&vl53l1x);
//     uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "%u %u", __LINE__, status);
    status = VL53L1_StaticInit(&vl53l1x);
//     uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "%u %u", __LINE__, status);
    status = VL53L1_SetDistanceMode(&vl53l1x, VL53L1_DISTANCEMODE_SHORT);
//     uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "%u %u", __LINE__, status);
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&vl53l1x, 100000);
//     uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "%u %u", __LINE__, status);
    status = VL53L1_SetInterMeasurementPeriodMilliSeconds(&vl53l1x, 1);
//     uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "%u %u", __LINE__, status);
    status = VL53L1_StartMeasurement(&vl53l1x);
//     uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "%u %u", __LINE__, status);
    
    
    
    worker_thread_add_timer_task(&WT, &vl53l1x_test_task, vl53l1x_test_task_func, NULL, LL_US2ST(500), true);
}

static void vl53l1x_test_task_func(struct worker_thread_timer_task_s* task) {
    (void)task;
    
    VL53L1_Error status;
    uint8_t measurement_available = 0;
    status = VL53L1_GetMeasurementDataReady(&vl53l1x, &measurement_available);
//     uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "%u %u", __LINE__, status);
    if (measurement_available) {
        VL53L1_RangingMeasurementData_t meas_data;
        status = VL53L1_GetRangingMeasurementData(&vl53l1x, &meas_data);
//         uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "%u %u", __LINE__, status);
        status = VL53L1_ClearInterruptAndStartMeasurement(&vl53l1x);
//         uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "%u %u", __LINE__, status);
        
        if (status == VL53L1_ERROR_NONE) {
            if (meas_data.RangeStatus == 0 || meas_data.RangeStatus == 7) {
                uavcan_send_debug_keyvalue("rng", meas_data.RangeMilliMeter*1e-3);
            } else {
                uavcan_send_debug_keyvalue("stat", meas_data.RangeStatus);
                uavcan_send_debug_keyvalue("rng", -1);
            }
        }
    }
}
