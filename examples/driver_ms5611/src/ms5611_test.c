#include <hal.h>
#include <modules/driver_ms5611/driver_ms5611.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/uavcan_debug/uavcan_debug.h>
#include <modules/pubsub/pubsub.h>
#include <uavcan.equipment.air_data.StaticPressure.h>
#include <uavcan.equipment.air_data.StaticTemperature.h>
#include <modules/timing/timing.h>

#define WT hpwork_thread
WORKER_THREAD_DECLARE_EXTERN(WT)

static struct ms5611_instance_s ms5611;
static struct pubsub_topic_s ms5611_topic;
static struct pubsub_listener_s ms5611_listener;
static struct worker_thread_timer_task_s ms5611_task;
static void ms5611_task_func(struct worker_thread_timer_task_s* task);
static struct uavcan_equipment_air_data_StaticTemperature_s temp;
static struct uavcan_equipment_air_data_StaticPressure_s press;

static void ms5611_sample_handler(size_t msg_size, const void* msg, void* ctx) {
    (void)msg_size;
    (void)ctx;
    const struct ms5611_sample_s* sample = (const struct ms5611_sample_s*)msg;
    press.static_pressure = sample->pressure_pa;
    temp.static_temperature = sample->temperature_K - 273.15f;
    uavcan_broadcast(0, &uavcan_equipment_air_data_StaticPressure_descriptor, CANARD_TRANSFER_PRIORITY_HIGH, &press);
    uavcan_broadcast(0, &uavcan_equipment_air_data_StaticTemperature_descriptor, CANARD_TRANSFER_PRIORITY_HIGH, &temp);
}

RUN_AFTER(INIT_END) {
    pubsub_init_topic(&ms5611_topic, NULL);
    pubsub_listener_init_and_register(&ms5611_listener, &ms5611_topic, ms5611_sample_handler, NULL);
    ms5611_init(&ms5611, 3, BOARD_PAL_LINE_SPI3_MS5611_CS, &WT, &ms5611_topic);
    worker_thread_add_timer_task(&WT, &ms5611_task, ms5611_task_func, NULL, chTimeMS2I(10), true);
}

static void ms5611_task_func(struct worker_thread_timer_task_s* task) {
    (void)task;
    pubsub_listener_handle_until_timeout(&ms5611_listener, TIME_IMMEDIATE);
}
