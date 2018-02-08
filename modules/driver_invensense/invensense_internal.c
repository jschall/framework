#include "invensense_internal.h"

uint8_t invensense_get_whoami(enum invensense_imu_type_t imu_type) {
    switch(imu_type) {
        case INVENSENSE_IMU_TYPE_MPU6000:
            return 0x68;
        case INVENSENSE_IMU_TYPE_MPU6500:
            return 0x70;
        case INVENSENSE_IMU_TYPE_MPU9250:
            return 0x71;
        case INVENSENSE_IMU_TYPE_MPU9255:
            return 0x73;
        case INVENSENSE_IMU_TYPE_ICM20608:
            return 0xaf;
        case INVENSENSE_IMU_TYPE_ICM20602:
            return 0x12;
    }
}
