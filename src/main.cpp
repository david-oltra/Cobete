#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <driver/i2c.h>
#include "bmi160.h"
#include "bmi160_defs.h"

/* local macro definitions */
#define BMI160_INTERFACE_I2C  BMI160_I2C_INTF

/*! bmi160 Device address */
#define BMI160_DEV_ADDR       BMI160_I2C_ADDR

/*********************************************************************/
/* global variables */
/*********************************************************************/

/*! @brief This structure containing relevant bmi160 info */
struct bmi160_dev bmi160dev;

/*! @brief variable to hold the bmi160 accel data */
struct bmi160_sensor_data bmi160_accel;

/*! @brief variable to hold the bmi160 gyro data */
struct bmi160_sensor_data bmi160_gyro;

static const char *TAG = "COBETE";

extern "C" void app_main();

void init_i2c()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_21;
    conf.scl_io_num = GPIO_NUM_22;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    conf.clk_flags = 0;

    i2c_param_config(I2C_NUM_0, &conf);

    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    
}

static int8_t bmi_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data,
                                     uint16_t len)
{
    i2c_master_write_read_device(I2C_NUM_0,
                                    dev_addr,
                                    &reg_addr,
                                    1,
                                    data,
                                    len,
                                    pdMS_TO_TICKS(500)); 

    return BMI160_OK;
}
static int8_t bmi_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* read_data,
                                     uint16_t len)
{
    uint8_t buf[len +1];
    buf[0] = reg_addr;
    for (uint16_t i=0; i < len; i++)
    {
        buf[i+1] = read_data[i];
    }
    i2c_master_write_to_device(I2C_NUM_0,
                                dev_addr,
                                buf,
                                len+1,
                                pdMS_TO_TICKS(500)
                                );


    return BMI160_OK;
}

void bmi_delay(uint32_t period)
{
    vTaskDelay(pdMS_TO_TICKS(period));
}

void init_bmi160()
{
    bmi160dev.intf = BMI160_I2C_INTF;
    bmi160dev.id = BMI160_DEV_ADDR;
    bmi160dev.read = (bmi160_read_fptr_t)bmi_read;
    bmi160dev.write = (bmi160_write_fptr_t)bmi_write;
    bmi160dev.delay_ms = (bmi160_delay_fptr_t)bmi_delay;

    int16_t rslt;

    rslt = bmi160_init(&bmi160dev);

    printf("rslt: %X\n\n",rslt);


    if (rslt == BMI160_OK)
    {
        printf("BMI160 initialization success !\n");
        printf("Chip ID 0x%X\n", bmi160dev.chip_id);
    }
    else
    {
        printf("BMI160 initialization failure !\n");
    }

     /* Select the Output data rate, range of accelerometer sensor */
    bmi160dev.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
    bmi160dev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    bmi160dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    bmi160dev.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    bmi160dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    bmi160dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi160dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&bmi160dev);
    printf("rslt: %X\n\n",rslt);
}

void i2c_scanner()
{
        int i;
        esp_err_t espRc;
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
        printf("00:         ");
        for (i=3; i< 0x78; i++) {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
            i2c_master_stop(cmd);

            espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
            if (i%16 == 0) {
                printf("\n%.2x:", i);
            }
            if (espRc == 0) {
                printf(" %.2x", i);
            } else {
                printf(" --");
            }
            i2c_cmd_link_delete(cmd);
	}
	printf("\n");
	vTaskDelete(NULL);
}



void app_main() 
{
    init_i2c();
    init_bmi160();

    while(0)
    {
        i2c_scanner();
    }

    while(1)
    {
        /* To read both Accel and Gyro data */
        bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &bmi160_accel, &bmi160_gyro, &bmi160dev);

        printf("ax:%d\tay:%d\taz:%d\n", bmi160_accel.x, bmi160_accel.y, bmi160_accel.z);
        printf("gx:%d\tgy:%d\tgz:%d\n", bmi160_gyro.x, bmi160_gyro.y, bmi160_gyro.z);
        fflush(stdout);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
   
    

}