#include "driver_invensense.h"
#include "invensense_internal.h"

#ifdef MODULE_UAVCAN_DEBUG_ENABLED
#include <modules/uavcan_debug/uavcan_debug.h>
#define ICM_DEBUG(...) uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "ICM", __VA_ARGS__)
#else
#define ICM_DEBUG(...) {}
#endif

//CORE FUNCTIONS
static void invensense_read(struct invensense_instance_s* instance, uint8_t reg, size_t len, void* buf);
static void invensense_write(struct invensense_instance_s* instance, uint8_t reg, size_t len, void* buf);

//SENSOR DATA
static float invensense_get_temp(struct invensense_instance_s* instance);
static gyro_data_t invensense_get_gyro(struct invensense_instance_s* instance);

//FIFO
static void invensense_fifo_enable(struct invensense_instance_s* instance, fifo_setting_t fifo_setting);
static void invensense_fifo_disable(struct invensense_instance_s* instance);
static uint16_t invensense_get_fifo_count(struct invensense_instance_s* instance);

interrupt_status_t invensense_ISR(struct invensense_instance_s* instance, bool fifo_watermark_enabled);

//DEBUG
static void invensense_print_accel_offsets(struct invensense_instance_s* instance);
static void invensense_debug_print_reg(struct invensense_instance_s* instance, uint8_t reg_addr, uint8_t regs_to_read);

//SETUP
static void invensense_set_accel_offsets(struct invensense_instance_s* instance);
static void invensense_set_sensor_config(struct invensense_instance_s* instance, accel_scale_t accel_scale, gyro_scale_t gyro_scale, uint8_t ODR);
static void invensense_set_device_config(struct invensense_instance_s* instance);


void invensense_device_reset(struct invensense_instance_s* instance);

accel_scale_t accel_scale;
gyro_scale_t gyro_scale;


void invensense_init(struct invensense_instance_s* instance, uint8_t spi_idx, uint32_t select_line, enum invensense_imu_type_t imu_type, accel_scale_t accel_scale_g, gyro_scale_t gyro_scale_dps, uint8_t ODR) {
    if (!instance) {
        return;
    }

    spi_device_init(&instance->spi_dev, spi_idx, select_line, 10000, 8, SPI_DEVICE_FLAG_CPHA|SPI_DEVICE_FLAG_CPOL);

    uint8_t whoami = 0;
    switch(imu_type) {
        case INVENSENSE_IMU_TYPE_ICM20602:
            invensense_read(instance, INVENSENSE_REG_WHO_AM_I, sizeof(uint8_t), &whoami);
            ICM_DEBUG("whoami = 0x%02X", whoami);

            if(whoami == 0x12){
                spi_device_set_max_speed_hz(&instance->spi_dev, 10000000);  //If the expected device is connected, set the baud for 10MHz
            }
            else{
                ICM_DEBUG("Device not found");
            }
            break;

        default:
            ICM_DEBUG("Invalid IMU type");
            break;
    }

    //Reset device (before configuring)
    invensense_device_reset(instance);
    chThdSleep(MS2ST(100));

    //Configure Device and Sensors (scale, ODR)
    invensense_set_device_config(instance);
    invensense_set_sensor_config(instance, accel_scale_g, gyro_scale_dps, ODR);

    //DEBUG
//     invensense_debug_print_reg(instance, INVENSENSE_REG_SMPLRT_DIV, 1);
//     invensense_debug_print_reg(instance, INVENSENSE_REG_GYRO_CONFIG, 1);
//     invensense_debug_print_reg(instance, INVENSENSE_REG_ACCEL_CONFIG1, 1);

    //Get temperature
    float temperature = invensense_get_temp(instance);
    ICM_DEBUG("temperature %d", (int32_t)temperature*100);

//     bool read_gyro_dyn = 0;
//     if(read_gyro_dyn)
//     {
//         gyro_data_t gyro_data = invensense_get_gyro(instance);
//         ICM_DEBUG("gyro_x %.1f", (double)gyro_data.x);
//         ICM_DEBUG("gyro_y %.1f", (double)gyro_data.y);
//         ICM_DEBUG("gyro_z %.1f", (double)gyro_data.z);
//         while (true) {
//             gyro_data_t gyro_data = invensense_get_gyro(instance);
//             ICM_DEBUG("gyro %.1f %.1f %.1f", (double)gyro_data.x, (double)gyro_data.y, (double)gyro_data.z);
//             chThdSleep(MS2ST(200));
//         }
//     }

//     invensense_print_accel_offsets(instance);
//     chThdSleep(S2ST(1));
//     invensense_set_accel_offsets(instance);
//     chThdSleep(S2ST(1));
//     invensense_print_accel_offsets(instance);
}


/////////////
//FUNCTIONS//
/////////////
//READ
static void invensense_read(struct invensense_instance_s* instance, uint8_t reg, size_t len, void* buf)
{
    reg |= 0x80;

    spi_device_begin(&instance->spi_dev);
    spi_device_send(&instance->spi_dev, sizeof(reg), &reg);
    spi_device_receive(&instance->spi_dev, len, buf);
    spi_device_end(&instance->spi_dev);
}

//WRITE
static void invensense_write(struct invensense_instance_s* instance, uint8_t reg, size_t len, void* buf)
{
    spi_device_begin(&instance->spi_dev);
    spi_device_send(&instance->spi_dev, sizeof(reg), &reg);
    spi_device_send(&instance->spi_dev, len, buf);
    spi_device_end(&instance->spi_dev);
}


/////////////////
//CONFIGURATION//
/////////////////
//SETUP DEVICE
static void invensense_set_device_config(struct invensense_instance_s* instance)
{
    //Setup device config
    invensense_debug_print_reg(instance, INVENSENSE_REG_CONFIG, 1);
    uint8_t reg_config_out = 0;    //FIFO will overwrite oldest data with newest data if buffer filled
    invensense_write(instance, INVENSENSE_REG_CONFIG, sizeof(uint8_t), &reg_config_out);
    invensense_debug_print_reg(instance, INVENSENSE_REG_CONFIG, 1);

    //Setup power management
    invensense_debug_print_reg(instance, INVENSENSE_REG_PWR_MGMT_1, 1);
    uint8_t reg_pwr1_out = 0x01;    //No sleep mode, select best clock
    invensense_write(instance, INVENSENSE_REG_PWR_MGMT_1, sizeof(uint8_t), &reg_pwr1_out);
    invensense_debug_print_reg(instance, INVENSENSE_REG_PWR_MGMT_1, 1);
}

//SETUP SENSORS
static void invensense_set_sensor_config(struct invensense_instance_s* instance, accel_scale_t accel_scale_g, gyro_scale_t gyro_scale_dps, uint8_t ODR)
{
    ICM_DEBUG("accel 0x%02x", accel_scale_g);
    ICM_DEBUG("gyro 0x%02x", gyro_scale_dps);

    //Configure gyroscope
    uint8_t gyro_config_out = 0;
    switch(gyro_scale_dps)
    {
        case GYRO_FS_250dps:
            gyro_config_out = BIT_GYRO_FS_250dps;   //Sets register to 0x00 (no self test)
            invensense_write(instance, INVENSENSE_REG_GYRO_CONFIG, sizeof(uint8_t), &gyro_config_out);
            gyro_scale = GYRO_FS_250dps;    //Set global gyro scale variable to relfect register
            break;

        case GYRO_FS_500dps:
            gyro_config_out = BIT_GYRO_FS_500dps;
            invensense_write(instance, INVENSENSE_REG_GYRO_CONFIG, sizeof(uint8_t), &gyro_config_out);
            gyro_scale = GYRO_FS_500dps;
            break;

        case GYRO_FS_1000dps:
            gyro_config_out = BIT_GYRO_FS_1000dps;
            invensense_write(instance, INVENSENSE_REG_GYRO_CONFIG, sizeof(uint8_t), &gyro_config_out);
            gyro_scale = GYRO_FS_1000dps;
            break;

        case GYRO_FS_2000dps:
            gyro_config_out = BIT_GYRO_FS_2000dps;
            invensense_write(instance, INVENSENSE_REG_GYRO_CONFIG, sizeof(uint8_t), &gyro_config_out);
            gyro_scale = GYRO_FS_2000dps;
            break;

        default:
            break;
    }


    //Configure accelerometer
    uint8_t accel_config_out = 0;
    switch (accel_scale_g)
    {
        case ACCEL_FS_2g:
            accel_config_out = BIT_ACCEL_FS_2g;     //Sets register to 0x00 (no self test)
            invensense_write(instance, INVENSENSE_REG_ACCEL_CONFIG1, sizeof(uint8_t), &accel_config_out);
            accel_scale = ACCEL_FS_2g;      //set global accel scale variable to reflect register
            break;

        case ACCEL_FS_4g:
            accel_config_out = BIT_ACCEL_FS_4g;
            invensense_write(instance, INVENSENSE_REG_ACCEL_CONFIG1, sizeof(uint8_t), &accel_config_out);
            accel_scale = ACCEL_FS_4g;
            break;

        case ACCEL_FS_8g:
            accel_config_out = BIT_ACCEL_FS_8g;     //Sets register to 0x00 (no self test)
            invensense_write(instance, INVENSENSE_REG_ACCEL_CONFIG1, sizeof(uint8_t), &accel_config_out);
            accel_scale = ACCEL_FS_8g;
            break;

        case ACCEL_FS_16g:
            accel_config_out = BIT_ACCEL_FS_16g;     //Sets register to 0x00 (no self test)
            invensense_write(instance, INVENSENSE_REG_ACCEL_CONFIG1, sizeof(uint8_t), &accel_config_out);
            accel_scale = ACCEL_FS_16g;
            break;

        default:
            break;
    }

    //Set output data rate (ODR)
    invensense_write(instance, INVENSENSE_REG_SMPLRT_DIV, sizeof(uint8_t), &ODR);
}


///////////////
//SENSOR DATA//
///////////////

//READ TEMP
float invensense_get_temp(struct invensense_instance_s* instance)
{

    uint8_t reg_temp[2];
    invensense_read(instance, INVENSENSE_REG_TEMP_OUT_H, sizeof(uint16_t), reg_temp);    //should read two bytes into temp[0] and temp[1] (?)
    int16_t temperature = (reg_temp[1] << 8) | (reg_temp[0]);
    float temp_out = (float)(temperature/326.8 + 25);
    return temp_out;
}


//READ GYRO (gyro_t)
static gyro_data_t invensense_get_gyro(struct invensense_instance_s* instance)
{
    uint8_t reg_gyro[6];
    invensense_read(instance, INVENSENSE_REG_GYRO_XOUT_H, 6, reg_gyro);

    int16_t data_x = (reg_gyro[0] << 8)|(reg_gyro[1]);
    int16_t data_y = (reg_gyro[2] << 8)|(reg_gyro[3]);
    int16_t data_z = (reg_gyro[4] << 8)|(reg_gyro[5]);

    gyro_data_t data_out;
    switch(gyro_scale)
    {
        case GYRO_FS_250dps:
            data_out.x = (float)250*data_x/32767;
            data_out.y = (float)250*data_y/32767;
            data_out.z = (float)250*data_z/32767;
            break;

        case GYRO_FS_500dps:
            data_out.x = (float)500*data_x/32767;
            data_out.y = (float)500*data_y/32767;
            data_out.z = (float)500*data_z/32767;
            break;

        case GYRO_FS_1000dps:
            data_out.x = (float)1000*data_x/32767;
            data_out.y = (float)1000*data_y/32767;
            data_out.z = (float)1000*data_z/32767;
            break;

        case GYRO_FS_2000dps:
            data_out.x = (float)2000*data_x/32767;
            data_out.y = (float)2000*data_y/32767;
            data_out.z = (float)2000*data_z/32767;
            break;

        default:
            break;

    }
    return data_out;
}

//READ ACCEL (accel_t)
accel_data_t invensense_get_accel(struct invensense_instance_s* instance)
{
    uint8_t reg_accel[6];
    invensense_read(instance, INVENSENSE_REG_ACCEL_XOUT_H, 6, reg_accel);

    int16_t data_x = (reg_accel[0] << 8)|(reg_accel[1]);
    int16_t data_y = (reg_accel[2] << 8)|(reg_accel[3]);
    int16_t data_z = (reg_accel[4] << 8)|(reg_accel[5]);

    accel_data_t data_out;
    switch(accel_scale)
    {
        case ACCEL_FS_2g:
            data_out.x = 9.80655f*2*data_x/32767;
            data_out.y = 9.80655f*2*data_y/32767;
            data_out.z = 9.80655f*2*data_z/32767;
            break;

        case ACCEL_FS_4g:
            data_out.x = 9.80655f*4*data_x/32767;
            data_out.y = 9.80655f*4*data_y/32767;
            data_out.z = 9.80655f*4*data_z/32767;
            break;

        case ACCEL_FS_8g:
            data_out.x = 9.80655f*8*data_x/32767;
            data_out.y = 9.80655f*8*data_y/32767;
            data_out.z = 9.80655f*8*data_z/32767;
            break;

        case ACCEL_FS_16g:
            data_out.x = 9.80655f*16*data_x/32767;
            data_out.y = 9.80655f*16*data_y/32767;
            data_out.z = 9.80655f*16*data_z/32767;
            break;

        default:
            break;
    }
    return data_out;
}

static void invensense_fifo_disable(struct invensense_instance_s* instance)
{
    uint8_t reg_user_ctrl_out = 0x00;
    invensense_write(instance, INVENSENSE_REG_USER_CTRL, sizeof(uint8_t), &reg_user_ctrl_out);
}

static void invensense_fifo_enable(struct invensense_instance_s* instance, fifo_setting_t fifo_setting)
{
    uint8_t reg_user_ctrl_out = 0x00;
    reg_user_ctrl_out = BIT_FIFO_EN;
    invensense_write(instance, INVENSENSE_REG_USER_CTRL, sizeof(uint8_t), &reg_user_ctrl_out);

    uint8_t reg_fifo_en_out = 0x00;
    switch(fifo_setting)
    {
        case ACCEL_FIFO_EN:
            reg_fifo_en_out = BIT_ACCEL_FIFO_EN;
            break;

        case GYRO_FIFO_EN:
            reg_fifo_en_out = BIT_GYRO_FIFO_EN;
            break;

        case ACCEL_GYRO_FIFO_EN:
            reg_fifo_en_out = BIT_ACCEL_FIFO_EN | BIT_GYRO_FIFO_EN;
            break;

        default:
            break;
    }

    invensense_write(instance, INVENSENSE_REG_FIFO_EN, sizeof(uint8_t), &reg_fifo_en_out);
}

static uint16_t invensense_get_fifo_count(struct invensense_instance_s* instance)
{
    uint8_t reg_fifo_count[2];
    invensense_read(instance, INVENSENSE_REG_FIFO_COUNTH, 2, reg_fifo_count);
    uint16_t fifo_count = (reg_fifo_count[0] << 8)|(reg_fifo_count[1]);
    return fifo_count;
}


static uint8_t invensense_get_interrupt_status(struct invensense_instance_s* instance)
{
    uint8_t reg_int_status = 0;
    invensense_read(instance, INVENSENSE_REG_INT_STATUS, sizeof(uint8_t), &reg_int_status);
    return reg_int_status;
}

void invensense_device_reset(struct invensense_instance_s* instance) {
    uint8_t pwr_mgmt_1_reg_reset = BIT_DEVICE_RESET;
    invensense_write(instance, INVENSENSE_REG_PWR_MGMT_1, sizeof(uint8_t), &pwr_mgmt_1_reg_reset);
}

////////////////////
//INTERRUPT VECTOR//
////////////////////
interrupt_status_t invensense_ISR(struct invensense_instance_s* instance, bool fifo_watermark_enabled)
{
    //GENERATE TIMESTAMP, save where thread can access later
    interrupt_status_t interrupt_status;

    interrupt_status.TIMESTAMP = 0; //set to generated timestamp (above)

    if(fifo_watermark_enabled)
    {
        invensense_read(instance, INVENSENSE_REG_FIFO_WM_INT_STATUS, sizeof(uint8_t), &interrupt_status.INT_FIFO_WM);
    }
    else
    {
        interrupt_status.INT_FIFO_WM = 0;
    }

    invensensense_read(instance, INVENSENSE_REG_INT_STATUS, sizeof(uint8_t), &interrupt_status.INT_STATUS);

    return interrupt_status;
}


/////////
//DEBUG//
/////////
static void invensense_debug_print_reg(struct invensense_instance_s* instance, uint8_t reg_addr, uint8_t regs_to_read)
{
    uint8_t reg_value[regs_to_read];
    invensense_read(instance, reg_addr, regs_to_read, reg_value);

    for(uint8_t i = 0; i < regs_to_read; i++)
    {
        ICM_DEBUG("Register = 0x%02x", reg_value[i]);
    }
}

static void invensense_print_accel_offsets(struct invensense_instance_s* instance)
{
    uint8_t reg_accel_offset[6];
    invensense_read(instance, INVENSENSE_REG_XA_OFFSET_H, 6, reg_accel_offset);

    int16_t accel_offset_x = (reg_accel_offset[0] << 8)|(reg_accel_offset[1]);
    int16_t accel_offset_y = (reg_accel_offset[2] << 8)|(reg_accel_offset[3]);
    int16_t accel_offset_z = (reg_accel_offset[4] << 8)|(reg_accel_offset[5]);

    ICM_DEBUG("acc_off_x = %d", accel_offset_x);
    ICM_DEBUG("acc_off_y = %d", accel_offset_y);
    ICM_DEBUG("acc_off_z = %d", accel_offset_z);
}



static void invensense_set_accel_offsets(struct invensense_instance_s* instance)
{
    uint8_t reg_accel_offset_out[6];
//     reg_accel_offset_out[0] = 0x1A;
//     reg_accel_offset_out[1] = 0x2B;
//     reg_accel_offset_out[2] = 0x3C;
//     reg_accel_offset_out[3] = 0x4D;
//     reg_accel_offset_out[4] = 0x5E;
//     reg_accel_offset_out[5] = 0x6F;

    reg_accel_offset_out[0] = 0;
    reg_accel_offset_out[1] = 0;
    reg_accel_offset_out[2] = 0;
    reg_accel_offset_out[3] = 0;
    reg_accel_offset_out[4] = 0;
    reg_accel_offset_out[5] = 0;

    ICM_DEBUG("x bytes: %02x%02x", reg_accel_offset_out[1], reg_accel_offset_out[0]);
    ICM_DEBUG("y bytes: %02x%02x", reg_accel_offset_out[3], reg_accel_offset_out[2]);
    ICM_DEBUG("z bytes: %02x%02x", reg_accel_offset_out[5], reg_accel_offset_out[4]);

    invensense_write(instance, INVENSENSE_REG_XA_OFFSET_H, 6, reg_accel_offset_out);
    //chThdSleep(S2ST(1));    //Sleep 1 sec
}








