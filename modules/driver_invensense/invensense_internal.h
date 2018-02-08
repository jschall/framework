#pragma once

#include "driver_invensense.h"

//#define INVENSENSE_REG_WHOAMI 0x75


//Data Output Registers


//Register Map
#define INVENSENSE_REG_XG_OFFS_TC_H         0x04
#define INVENSENSE_REG_YG_OFFS_TC_H         0x07
#define INVENSENSE_REG_ZG_OFFS_TC_H         0x0A

#define INVENSENSE_REG_SELF_TEST_X_ACCEL    0x0D
#define INVENSENSE_REG_SELF_TEST_Y_ACCEL    0x0E
#define INVENSENSE_REG_SELF_TEST_Z_ACCEL    0x0F

#define INVENSENSE_REG_XG_OFF_USRH          0x13    //X-axis gyro offset MSB, added to data before it's put into GYRO_XOUT
#define INVENSENSE_REG_YG_OFF_USRH          0x15    //Y-axis gyro offset
#define INVENSENSE_REG_ZG_OFF_USRH          0x17    //Z-axis gyro offset
#define INVENSENSE_REG_SMPLRT_DIV           0x19    //Sample rate divider: SR = 1kHz/(1+SMPLRT_DIV)
// #   define ODR_10Hz                         0x63
// #   define ODR_50Hz                         0x13
// #   define ODR_100Hz                        0X09
// #   define ODR_200Hz                        0x04
// #   define ODR_250Hz                        0x03
// #   define ODR_500Hz                        0x01

#define INVENSENSE_REG_CONFIG               0x1A
#   define BIT_FIFO_NO_OVERWRITE            0x40
#   define BIT_DEVICE_DISABLE               0x80

#define INVENSENSE_REG_GYRO_CONFIG          0x1B    //Gyro config
//bits 1:0 for FCHOICE_B to bypass DLPF
#   define BIT_GYRO_FS_250dps               0x00
#   define BIT_GYRO_FS_500dps               0x08
#   define BIT_GYRO_FS_1000dps              0x10
#   define BIT_GYRO_FS_2000dps              0x18
#   define BIT_GYRO_Z_ST                    0x20
#   define BIT_GYRO_Y_ST                    0x40
#   define BIT_GYRO_X_ST                    0x80

#define INVENSENSE_REG_ACCEL_CONFIG1        0x1C    //Accel config
#   define BIT_ACCEL_FS_2g                  0x00
#   define BIT_ACCEL_FS_4g                  0x08
#   define BIT_ACCEL_FS_8g                  0x10
#   define BIT_ACCEL_FS_16g                 0x18
#   define BIT_ACCEL_Z_ST                   0x20
#   define BIT_ACCEL_Y_ST                   0x40
#   define BIT_ACCEL_X_ST                   0x80     

#define INVENSENSE_REG_ACCEL_CONFIG2        0x1D    //Accel config: LP mode


#define INVENSENSE_REG_LP_MODE_CFG          0x1E    
#define INVENSENSE_REG_ACCEL_WOM_X_THR      0x20
#define INVENSENSE_REG_ACCEL_WOM_Y_THR      0x21
#define INVENSENSE_REG_ACCEL_WOM_Z_THR      0x22

#define INVENSENSE_REG_FIFO_EN              0x23
#   define BIT_GYRO_FIFO_EN                 0x10    //Write gyro and temp data to fifo
#   define BIT_ACCEL_FIFO_EN                0x80    //Write accel and temp data to fifo

#define INVENSENSE_REG_FSYNC_INT            0x36    //Bit 7 set when FSYNC interrupt has been generated; RTC (read to clear)

#define INVENSENSE_REG_INT_PIN_CFG          0x37    //Configure interrupt pins
#   define BIT_INT_LEVEL_LOW                0x80    //Set to make DRDY pin active LOW 
#   define BIT_INT_OPEN                     0x40    //Set to config DRDY as open drain
#   define BIT_INT_LATCH_EN                 0x20    //Set to hold interrupt until status cleared
#   define BIT_INT_RD_CLEAR_ANY             0x10    //Set to clear interrupt status when ANY register is read (as opposed to when INT_STATUS reg is read)
#   define BIT_FSYNC_INT_LEVEL_LOW          0x08    //Set to make FSYNC pin active LOW 
#   define BIT_INT_MODE_EN                  0x04    //Set to enable FSYNC interrupt

#define INVENSENSE_REG_INT_ENABLE           0x38    //WOM, FIFO overflow, etc.

#define INVENSENSE_REG_FIFO_WM_INT_STATUS   0x39    //Interrupt status (FIFO watermark)
#   define  BIT_FIFO_WM_INT                 0x40

#define INVENSENSE_REG_INT_STATUS           0x3A    //Interrupt status (WOM, FIFO, GDRIVE, DRDY)
#   define BIT_FIFO_OFLOW_INT               0x10
#   define BIT_GDRIVE_INT                   0x04
#   define BIT_DATA_RDY_INT                 0x01

#define INVENSENSE_REG_ACCEL_XOUT_H         0x3B
#define INVENSENSE_REG_ACCEL_YOUT_H         0x3D
#define INVENSENSE_REG_ACCEL_ZOUT_H         0x3F
#define INVENSENSE_REG_TEMP_OUT_H           0x41
#define INVENSENSE_REG_GYRO_XOUT_H          0x43
#define INVENSENSE_REG_GYRO_YOUT_H          0x45
#define INVENSENSE_REG_GYRO_ZOUT_H          0x47

#define INVENSENSE_REG_SELF_TEST_X_GYRO     0x50
#define INVENSENSE_REG_SELF_TEST_Y_GYRO     0x51
#define INVENSENSE_REG_SELF_TEST_Z_GYRO     0x52

#define INVENSENSE_REG_FIFO_WM_TH1          0x60
#define INVENSENSE_REG_FIFO_WM_TH2          0x61

#define INVENSENSE_REG_SIGNAL_PATH_RESET    0x68
#define INVENSENSE_REG_ACCEL_INTEL_CTRL     0x69
#define INVENSENSE_REG_USER_CTRL            0x6A
#   define BIT_FIFO_EN                      0x40
#   define BIT_FIFO_RST                     0x04

#define INVENSENSE_REG_PWR_MGMT_1           0x6B
#   define BIT_DEVICE_RESET                 0x80
#   define BIT_DEVICE_SLEEP                 0x40
#   define BIT_DEVICE_CYCLE                 0x20
#   define BIT_GYRO_STANDBY                 0x10
#   define BIT_TEMP_DISABLE                 0x08

#define INVENSENSE_REG_PWR_MGMT_2           0x6C
#define INVENSENSE_REG_I2C_IF               0x70
#define INVENSENSE_REG_FIFO_COUNTH          0x72
#define INVENSENSE_REG_FIFO_R_W             0x74
#define INVENSENSE_REG_WHO_AM_I             0x75

#define INVENSENSE_REG_XA_OFFSET_H          0x77
#define INVENSENSE_REG_YA_OFFSET_H          0x7A
#define INVENSENSE_REG_ZA_OFFSET_H          0x7D

uint8_t invensense_get_whoami(enum invensense_imu_type_t imu_type);
