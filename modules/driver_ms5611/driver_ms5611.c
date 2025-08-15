#include "driver_ms5611.h"
#include <common/helpers.h>
#include <modules/uavcan_debug/uavcan_debug.h>

#define MS5611_CMD_RESET                    0x1E
#define MS5611_CMD_CVT_D1_256               0x40
#define MS5611_CMD_CVT_D1_512               0x42
#define MS5611_CMD_CVT_D1_1024              0x44
#define MS5611_CMD_CVT_D1_2048              0x46
#define MS5611_CMD_CVT_D1_4096              0x48
#define MS5611_CMD_CVT_D2_256               0x50
#define MS5611_CMD_CVT_D2_512               0x52
#define MS5611_CMD_CVT_D2_1024              0x54
#define MS5611_CMD_CVT_D2_2048              0x56
#define MS5611_CMD_CVT_D2_4096              0x58
#define MS5611_CMD_ADC_READ                 0x00
#define MS5611_CMD_PROM_READ                0xA0

static void ms5611_task_func(struct worker_thread_timer_task_s* task);

static void ms5611_read_prom(struct ms5611_instance_s* instance);
static void ms5611_compute_temperature_and_pressure(struct ms5611_instance_s* instance, float* pressure_pa, float* temperature_K);
static uint32_t ms5611_read_adc(struct ms5611_instance_s* instance);
static void ms5611_cmd(struct ms5611_instance_s* instance, uint8_t cmd);
static void ms5611_read(struct ms5611_instance_s* instance, uint8_t addr, uint8_t n, uint8_t* buf);
static bool crc4(uint16_t *prom);

void ms5611_init(struct ms5611_instance_s* instance, uint8_t spi_idx, uint32_t select_line, struct worker_thread_s* worker_thread, struct pubsub_topic_s* topic) {
    instance->topic = topic;
    instance->worker_thread = worker_thread;

    // Ensure sufficient power-up time has elapsed
    chThdSleep(chTimeMS2I(100));

    spi_device_init(&instance->spi_dev, spi_idx, select_line, 20000000, 8, SPI_DEVICE_FLAG_CPHA|SPI_DEVICE_FLAG_CPOL);

    // Reset device
    ms5611_cmd(instance, MS5611_CMD_RESET);

    chThdSleep(chTimeMS2I(20));

    ms5611_read_prom(instance);

    instance->process_step = 0;
    ms5611_cmd(instance, MS5611_CMD_CVT_D2_1024);
    worker_thread_add_timer_task(instance->worker_thread, &instance->task, ms5611_task_func, instance, chTimeUS2I(2500), false);
}

static void ms5611_task_func(struct worker_thread_timer_task_s* task) {
    struct ms5611_instance_s* instance = (struct ms5611_instance_s*)worker_thread_task_get_user_context(task);

    switch(instance->process_step) {
        case 0: {
            // Read temperature and start pressure read
            instance->D2 = ms5611_read_adc(instance);
            ms5611_cmd(instance, MS5611_CMD_CVT_D1_1024);
            instance->conversion_start_time = chVTGetSystemTimeX();
            worker_thread_add_timer_task(instance->worker_thread, &instance->task, ms5611_task_func, instance, chTimeUS2I(2500), false);
            break;
        }
        case 1: {
            // Read pressure, publish temperature+pressure observation, start temperature read
            instance->D1 = ms5611_read_adc(instance);
            ms5611_cmd(instance, MS5611_CMD_CVT_D2_1024);

            if (instance->prom_read_ok) {
                struct ms5611_sample_s sample;
                sample.timestamp = instance->conversion_start_time;
                ms5611_compute_temperature_and_pressure(instance, &sample.pressure_pa, &sample.temperature_K);
                pubsub_publish_message(instance->topic, sizeof(sample), pubsub_copy_writer_func, &sample);
            }
            worker_thread_timer_task_reschedule(instance->worker_thread, &instance->task, chTimeMS2I(17));
            break;
        }
    }

    instance->process_step = (instance->process_step + 1) % 2;
}

static void ms5611_compute_temperature_and_pressure(struct ms5611_instance_s* instance, float* pressure_pa, float* temperature_K) {
    uint32_t D1 = instance->D1;
    uint32_t D2 = instance->D2;

    int32_t dT = D2 - ((uint32_t)instance->prom.s.c5_reference_temp << 8);

    int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * instance->prom.s.c6_temp_coeff_temp) / (1<<23));

    int64_t OFF = ((int64_t)instance->prom.s.c2_pressure_offset << 16) + (((int64_t)instance->prom.s.c4_temp_coeff_pres_offset * dT) / (1<<7));
    int64_t SENS = ((int64_t)instance->prom.s.c1_pressure_sens << 15) + (((int64_t)instance->prom.s.c3_temp_coeff_pres_sens * dT) / (1<<8));

    if (TEMP < 2000) {
        int32_t T2 = SQ((int64_t)dT) / (1<<31);
        int64_t OFF2 = 5 * SQ((int64_t)TEMP - 2000) / 2;
        int64_t SENS2 = 5 * SQ((int64_t)TEMP - 2000) / 4;

        if (TEMP < -1500) {
            OFF2 += 7 * SQ(TEMP + 1500);
            SENS2 += (11 * SQ(TEMP + 1500)) / 2;
        }

        TEMP -= T2;
        OFF  -= OFF2;
        SENS -= SENS2;
    }

    int32_t P = (((D1 * SENS) / (1<<21)) - OFF) / (1<<15);

    *pressure_pa = P;
    *temperature_K = TEMP*0.01f + 273.15f;
};

static void ms5611_read_prom(struct ms5611_instance_s* instance) {
    uint8_t val[3] = {0};
    uint8_t addr = MS5611_CMD_PROM_READ;

    instance->prom_read_ok = false;

    for (uint8_t i = 0; i < 8; i++) {
        ms5611_read(instance, addr, 2, val);
        addr += 2;
        instance->prom.c[i] = (val[0] << 8) | val[1];
        if (instance->prom.c[i] != 0) {
            instance->prom_read_ok = true;
        }
    }

    instance->prom_read_ok = instance->prom_read_ok && crc4(instance->prom.c);
}

static bool crc4(uint16_t *prom) {
    int16_t cnt;
    uint16_t n_rem;
    uint16_t crc_read;
    uint8_t n_bit;

    n_rem = 0x00;

    /* save the read crc */
    crc_read = prom[7];

    /* remove CRC byte */
    prom[7] = (uint16_t)(0xFF00 & (prom[7]));

    for (cnt = 0; cnt < 16; cnt++) {
        /* uneven bytes */
        if (cnt & 1) {
            n_rem ^= (uint8_t)((prom[cnt >> 1]) & 0x00FF);

        } else {
            n_rem ^= (uint8_t)(prom[cnt >> 1] >> 8);
        }

        for (n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000) {
                n_rem = (n_rem << 1) ^ 0x3000;

            } else {
                n_rem = (n_rem << 1);
            }
        }
    }

    /* final 4 bit remainder is CRC value */
    n_rem = (0x000F & (n_rem >> 12));
    prom[7] = crc_read;

    /* return true if CRCs match */
    return (uint16_t)(0x000F & crc_read) == (uint16_t)(n_rem ^ 0x00);
}

static uint32_t ms5611_read_adc(struct ms5611_instance_s* instance) {
    uint8_t val[3] = {0};
    ms5611_read(instance, MS5611_CMD_ADC_READ, 3, val);
    return (val[0] << 16) | (val[1] << 8) | val[2];
}

static void ms5611_cmd(struct ms5611_instance_s* instance, uint8_t cmd) {
    spi_device_begin(&instance->spi_dev);
    spi_device_send(&instance->spi_dev, sizeof(cmd), &cmd);
    spi_device_end(&instance->spi_dev);
}

static void ms5611_read(struct ms5611_instance_s* instance, uint8_t addr, uint8_t n, uint8_t* buf)
{
    spi_device_begin(&instance->spi_dev);
    spi_device_send(&instance->spi_dev, 1, &addr);
    spi_device_receive(&instance->spi_dev, n, buf);
    spi_device_end(&instance->spi_dev);
}


