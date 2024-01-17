#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <driver/i2c.h>
#include "bmi160.h"
#include "bmi160_defs.h"
#include <driver/mcpwm_prelude.h>
#include <cmath>

static TaskHandle_t _bmi160 = NULL;
static TaskHandle_t _sg90 = NULL;

#define BMI160_INTERFACE_I2C  BMI160_I2C_INTF
#define BMI160_DEV_ADDR       BMI160_I2C_ADDR

struct bmi160_dev bmi160dev;
struct bmi160_sensor_data bmi160_accel;
struct bmi160_sensor_data bmi160_gyro;

#define SENSITIVITY 500

#define SERVO_X_MIN_PULSEWIDTH_US 400  // Minimum pulse width in microsecond
#define SERVO_X_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_Y_MIN_PULSEWIDTH_US 400  // Minimum pulse width in microsecond
#define SERVO_Y_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum x_angle
#define SERVO_MAX_DEGREE        90    // Maximum x_angle
#define SERVO_LIMIT_X   15
#define SERVO_LIMIT_Y   30

#define SERVO_X_PULSE_GPIO             13        // GPIO connects to the PWM signal line
#define SERVO_Y_PULSE_GPIO             14        // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

static const char *TAG = "COBETE";

extern "C" void app_main();

void init()
{
    gpio_num_t pinout[3] = {GPIO_NUM_12, GPIO_NUM_26, GPIO_NUM_27};
    for (uint8_t i=0; i<3; i++){
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << pinout[i]);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);
        gpio_set_level(pinout[i],0);
    }
    gpio_set_level(GPIO_NUM_12,1);
    gpio_set_level(GPIO_NUM_27,1);
}

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

float x_deg = 0;
float y_deg = 0;

void bmi160(void *arg)
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
//    bmi160dev.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
//    bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
//    bmi160dev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    bmi160dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
//    bmi160dev.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
//    bmi160dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
//    bmi160dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi160dev.gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&bmi160dev);
    printf("rslt: %X\n\n",rslt);

    while(1)
    {
        float accel_z = 0;
        float accel_x = 0;
        float accel_y = 0;
        for (int i=0; i<SENSITIVITY; i++)
        {
            bmi160_get_sensor_data(BMI160_ACCEL_SEL, &bmi160_accel, 0, &bmi160dev);
        //    printf("ax:%d\tay:%d\taz:%d\n", bmi160_accel.x, bmi160_accel.y, bmi160_accel.z);printf("gx:%d\tgy:%d\tgz:%d\n", bmi160_gyro.x, bmi160_gyro.y, bmi160_gyro.z);
            fflush(stdout);
            accel_z = accel_z + bmi160_accel.z;
            accel_x = accel_x + bmi160_accel.x;
            accel_y = accel_y + bmi160_accel.y;
        }
        accel_z = accel_z / SENSITIVITY;
        accel_x = accel_x / SENSITIVITY;
        accel_y = accel_y / SENSITIVITY;
        
//        printf("accel_z = %f\n",accel_z);
//        printf("accel_y = %f\n",accel_y);
//        printf("accel_x = %f\n",accel_x);

        // accz = 1g * cos(θ)
        // accx= 1g * sin(θ) * cos(φ))
        //y_deg = acos(accel_z / 9.8);
        float phi_x = -atan(accel_y/accel_x);
        float phi_y = -atan(accel_y/accel_z);

        y_deg = phi_y*180/3.141592654;
        if(y_deg >= 0)
        {
            y_deg = y_deg-90;
        }
        else
        {
            y_deg = y_deg+90;
        }
        printf("y_deg = %f\n",y_deg);

        // accy / accx = -tan(φ)
        x_deg = phi_x*180/3.141592654;
        if(x_deg >= 0)
        {
            x_deg = x_deg-90;
        }
        else
        {
            x_deg = x_deg+90;
        }
        printf("x_deg = %f\n\n",x_deg);
    }
}

static inline uint32_t angle_to_compare(int angle, int SERVO_MAX_PULSEWIDTH_US, int SERVO_MIN_PULSEWIDTH_US)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

void sg90(void *arg)
{
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t X_timer = NULL;
    mcpwm_timer_config_t X_timer_config;
        X_timer_config.group_id = 0;
        X_timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
        X_timer_config.resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ;
        X_timer_config.period_ticks = SERVO_TIMEBASE_PERIOD;
        X_timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    
    ESP_ERROR_CHECK(mcpwm_new_timer(&X_timer_config, &X_timer));

    mcpwm_oper_handle_t X_oper = NULL;
    mcpwm_operator_config_t X_operator_config;
        X_operator_config.group_id = 0; // operator must be in the same group to the timer
    
    ESP_ERROR_CHECK(mcpwm_new_operator(&X_operator_config, &X_oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(X_oper, X_timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t X_comparator = NULL;
    mcpwm_comparator_config_t X_comparator_config;
        X_comparator_config.flags.update_cmp_on_tez = true;
    
    ESP_ERROR_CHECK(mcpwm_new_comparator(X_oper, &X_comparator_config, &X_comparator));

    mcpwm_gen_handle_t X_generator = NULL;
    mcpwm_generator_config_t X_generator_config;
        X_generator_config.gen_gpio_num = SERVO_X_PULSE_GPIO;
    
    ESP_ERROR_CHECK(mcpwm_new_generator(X_oper, &X_generator_config, &X_generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(X_comparator, angle_to_compare(0, SERVO_X_MAX_PULSEWIDTH_US, SERVO_X_MIN_PULSEWIDTH_US)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(X_generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(X_generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, X_comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(X_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(X_timer, MCPWM_TIMER_START_NO_STOP));


    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t Y_timer = NULL;
    mcpwm_timer_config_t Y_timer_config;
        Y_timer_config.group_id = 1;
        Y_timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
        Y_timer_config.resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ;
        Y_timer_config.period_ticks = SERVO_TIMEBASE_PERIOD;
        Y_timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    
    ESP_ERROR_CHECK(mcpwm_new_timer(&Y_timer_config, &Y_timer));

    mcpwm_oper_handle_t Y_oper = NULL;
    mcpwm_operator_config_t Y_operator_config;
        Y_operator_config.group_id = 1; // operator must be in the same group to the timer
    
    ESP_ERROR_CHECK(mcpwm_new_operator(&Y_operator_config, &Y_oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(Y_oper, Y_timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t Y_comparator = NULL;
    mcpwm_comparator_config_t Y_comparator_config;
        Y_comparator_config.flags.update_cmp_on_tez = true;
    
    ESP_ERROR_CHECK(mcpwm_new_comparator(Y_oper, &Y_comparator_config, &Y_comparator));

    mcpwm_gen_handle_t Y_generator = NULL;
    mcpwm_generator_config_t Y_generator_config;
        Y_generator_config.gen_gpio_num = SERVO_Y_PULSE_GPIO;
    
    ESP_ERROR_CHECK(mcpwm_new_generator(Y_oper, &Y_generator_config, &Y_generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(Y_comparator, angle_to_compare(0, SERVO_Y_MAX_PULSEWIDTH_US, SERVO_Y_MIN_PULSEWIDTH_US)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(Y_generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(Y_generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, Y_comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(Y_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(Y_timer, MCPWM_TIMER_START_NO_STOP));

    while (1) {
        if (x_deg>SERVO_LIMIT_X){x_deg = SERVO_LIMIT_X;}
        else if (x_deg<-SERVO_LIMIT_X){x_deg = -SERVO_LIMIT_X;}
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(X_comparator, angle_to_compare(x_deg, SERVO_X_MAX_PULSEWIDTH_US, SERVO_X_MIN_PULSEWIDTH_US)));
        if (y_deg>SERVO_LIMIT_Y){y_deg = SERVO_LIMIT_Y;}
        else if (y_deg<-SERVO_LIMIT_Y){y_deg = -SERVO_LIMIT_Y;}
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(Y_comparator, angle_to_compare(y_deg, SERVO_Y_MAX_PULSEWIDTH_US, SERVO_Y_MIN_PULSEWIDTH_US)));
        //Add delay, since it takes time for servo to rotate, usually 200ms/60degree rotation under 5V power supply
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main() 
{
    init();
    init_i2c();

    while(0)
    {
        i2c_scanner();
    }
   
    xTaskCreatePinnedToCore(bmi160, "bmi160", 4096, NULL, 9, &_bmi160, 1);    
    xTaskCreatePinnedToCore(sg90, "sg90", 4096, NULL, 10, &_sg90, 0);
    
}