#pragma once

#include <uavcan.equipment.gnss.Fix2.h>
#include <uavcan.equipment.gnss.Auxiliary.h>
#include <uavcan.equipment.ahrs.MagneticFieldStrength.h>
#include <modules/uavcan/uavcan.h>

enum sensor_type_t {
    SENSOR_TYPE_IMU,
    SENSOR_TYPE_GNSS,
    SENSOR_TYPE_MAG,
    NUM_SENSOR_TYPES
};

enum sensor_measurement_type_t {
    SENSOR_MEASUREMENT_TYPE_IMU_DELTA,
    SENSOR_MEASUREMENT_TYPE_GNSS_FIX2,
    SENSOR_MEASUREMENT_TYPE_GNSS_AUX,
    SENSOR_MEASUREMENT_TYPE_GNSS_ECEF,
    SENSOR_MEASUREMENT_TYPE_MAG_FIELD
};

#define _MEAS_TYPE_SIZES { \
    sizeof(struct sensor_measurement_imu_delta_s), \
    sizeof(struct sensor_measurement_gnss_fix2_s), \
    sizeof(struct sensor_measurement_gnss_aux_s), \
    sizeof(struct sensor_measurement_gnss_ecef_s), \
    sizeof(struct sensor_measurement_mag_field_s) \
}


#define _SENSOR_TYPE_NAMES { \
    "IMU",\
    "GNSS",\
    "MAG",\
}

struct sensor_measurement_s {
    uint32_t type;
    systime_t meas_time;
    uint8_t msg_body[];
};

struct sensor_measurement_imu_delta_s {
    float delta_time;
    float delta_angle[3];
    float delta_velocity[3];
};

struct sensor_measurement_gnss_fix2_s {
    struct uavcan_equipment_gnss_Fix2_s fix2;
};

struct sensor_measurement_gnss_aux_s {
    struct uavcan_equipment_gnss_Auxiliary_s aux;
};

struct sensor_measurement_gnss_ecef_s {
    double pos_ecef[3];
    float vel_ecef[3];
    float hAcc;
    float vAcc;
    float sAcc;
};

struct sensor_measurement_mag_field_s {
    struct uavcan_equipment_ahrs_MagneticFieldStrength_s mag;
};
