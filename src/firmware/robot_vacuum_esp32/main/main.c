#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/adc.h"

#include "esp_log.h"
#include "esp_timer.h"

// ======================================================
// LOG
// ======================================================
static const char *TAG = "SLAM_READY";

// ======================================================
// ROBOT PARAM
// ======================================================
#define WHEEL_RADIUS     0.021f
#define WHEEL_BASE       0.21f
#define PULSE_PER_REV    700.0f

#define MAX_V            0.15f
#define MAX_W            1.2f

#define PWM_MAX          255
#define PWM_MIN_START    130

#define DT               0.05f
#define DV_MAX           0.01f   // ramp m/s per cycle

// ======================================================
// MOTOR PIN
// ======================================================
#define IN1 25
#define IN2 26
#define ENA 14

#define IN3 33
#define IN4 32
#define ENB 27

// ======================================================
// ENCODER
// ======================================================
#define ENC_L_A 19
#define ENC_L_B 18
#define ENC_R_A 34
#define ENC_R_B 35

#define PCNT_L  PCNT_UNIT_0
#define PCNT_R  PCNT_UNIT_1

// ======================================================
// PWM
// ======================================================
#define LEDC_MODE  LEDC_LOW_SPEED_MODE
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_FREQ  1000
#define LEDC_RES   LEDC_TIMER_8_BIT
#define LEDC_CH_L  LEDC_CHANNEL_0
#define LEDC_CH_R  LEDC_CHANNEL_1

// ======================================================
// UART
// ======================================================
#define UART_NUM UART_NUM_1
#define TX_PIN   17
#define RX_PIN   16
#define BUF_SIZE 256

// ======================================================
// MPU6050
// ======================================================
#define I2C_NUM     I2C_NUM_0
#define I2C_SDA     21
#define I2C_SCL     22
#define I2C_FREQ    400000
#define MPU_ADDR    0x68

#define GYRO_ALPHA  0.98f

// ======================================================
// VACUUM MOTOR (ADC CONTROL)
// ======================================================
#define VAC_LPWM      15
#define VAC_RPWM      13
#define VAC_L_EN      23
#define VAC_R_EN      5

#define VAC_LED_MODE    LEDC_LOW_SPEED_MODE
#define VAC_LED_TIMER  LEDC_TIMER_1
#define VAC_LED_CH     LEDC_CHANNEL_2
#define VAC_PWM_FREQ   1500
#define VAC_PWM_RES    LEDC_TIMER_10_BIT
#define VAC_PWM_MAX    1023

#define VAC_ADC_CH     ADC1_CHANNEL_0   // GPIO36


// ======================================================
// PID (PI)
// ======================================================
typedef struct {
    float kp;
    float ki;
    float i;
} pi_t;

static pi_t piL = {.kp = 12.0f, .ki = 3.0f};
static pi_t piR = {.kp = 12.0f, .ki = 3.0f};

// ======================================================
// STATE
// ======================================================
static float vL_ref = 0, vR_ref = 0;
static float vL_cmd = 0, vR_cmd = 0;
static int pwmL = 0, pwmR = 0;
static int64_t last_cmd_time = 0;

// ======================================================
// ODOM
// ======================================================
typedef struct {
    float x;
    float y;
    float th;
} odom_t;

static odom_t odom = {0};
static int16_t lastL = 0, lastR = 0;

// ======================================================
// IMU
// ======================================================
static float gyro_bias_z = 0.0f;
static float gyro_z_lpf  = 0.0f;

// ======================================================
// MOTOR
// ======================================================
static inline void set_dir(int in1, int in2, int dir)
{
    if (dir > 0) {
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
    } else if (dir < 0) {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
    } else {
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 1); // brake
    }
}

void motor_init(void)
{
    gpio_set_direction(IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN3, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN4, GPIO_MODE_OUTPUT);

    ledc_timer_config_t t = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQ,
        .duty_resolution = LEDC_RES
    };
    ledc_timer_config(&t);

    ledc_channel_config_t c = {
        .speed_mode = LEDC_MODE,
        .timer_sel = LEDC_TIMER,
        .duty = 0
    };
    c.channel = LEDC_CH_L; c.gpio_num = ENA; ledc_channel_config(&c);
    c.channel = LEDC_CH_R; c.gpio_num = ENB; ledc_channel_config(&c);
}

void motor_set(int pwmL_, int pwmR_, int dirL, int dirR)
{
    if (pwmL_ < 0) pwmL_ = 0;
    if (pwmR_ < 0) pwmR_ = 0;
    if (pwmL_ > PWM_MAX) pwmL_ = PWM_MAX;
    if (pwmR_ > PWM_MAX) pwmR_ = PWM_MAX;

    set_dir(IN1, IN2, dirL);
    set_dir(IN3, IN4, dirR);

    ledc_set_duty(LEDC_MODE, LEDC_CH_L, pwmL_);
    ledc_update_duty(LEDC_MODE, LEDC_CH_L);
    ledc_set_duty(LEDC_MODE, LEDC_CH_R, pwmR_);
    ledc_update_duty(LEDC_MODE, LEDC_CH_R);
}

// ======================================================
// ENCODER
// ======================================================
void encoder_init(void)
{
    pcnt_config_t c = {
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768
    };

    c.pulse_gpio_num = ENC_L_A;
    c.ctrl_gpio_num  = ENC_L_B;
    c.unit = PCNT_L;
    pcnt_unit_config(&c);

    c.pulse_gpio_num = ENC_R_A;
    c.ctrl_gpio_num  = ENC_R_B;
    c.unit = PCNT_R;
    pcnt_unit_config(&c);

    pcnt_counter_clear(PCNT_L);
    pcnt_counter_clear(PCNT_R);
    pcnt_counter_resume(PCNT_L);
    pcnt_counter_resume(PCNT_R);
}
static inline void vacuum_set_pwm(int duty)
{
    if (duty < 0) duty = 0;
    if (duty > VAC_PWM_MAX) duty = VAC_PWM_MAX;

    ledc_set_duty(VAC_LED_MODE, VAC_LED_CH, duty);
    ledc_update_duty(VAC_LED_MODE, VAC_LED_CH);
}
void vacuum_init(void)
{
    // Enable driver
    gpio_set_direction(VAC_L_EN, GPIO_MODE_OUTPUT);
    gpio_set_direction(VAC_R_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(VAC_L_EN, 1);
    gpio_set_level(VAC_R_EN, 1);

    // PWM timer
    ledc_timer_config_t t = {
        .speed_mode = VAC_LED_MODE,
        .timer_num = VAC_LED_TIMER,
        .freq_hz = VAC_PWM_FREQ,
        .duty_resolution = VAC_PWM_RES,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&t);

    // PWM channel
    ledc_channel_config_t c = {
        .gpio_num = VAC_RPWM,
        .speed_mode = VAC_LED_MODE,
        .channel = VAC_LED_CH,
        .timer_sel = VAC_LED_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&c);

    // ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(VAC_ADC_CH, ADC_ATTEN_DB_11);

    ESP_LOGI("VACUUM", "Vacuum motor initialized");
}
void vacuum_task(void *arg)
{
    while (1) {
        int adc = adc1_get_raw(VAC_ADC_CH);   // 0–4095
        int duty = adc * VAC_PWM_MAX / 4095;

        vacuum_set_pwm(duty);

        ESP_LOGI("VACUUM", "ADC=%d  PWM=%d", adc, duty);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ======================================================
// I2C + IMU
// ======================================================
void i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ
    };
    i2c_param_config(I2C_NUM, &conf);
    i2c_driver_install(I2C_NUM, conf.mode, 0, 0, 0);
}

void mpu6050_init(void)
{
    uint8_t pwr[] = {0x6B, 0x00};
    i2c_master_write_to_device(I2C_NUM, MPU_ADDR, pwr, 2, pdMS_TO_TICKS(50));
}

static int16_t mpu_read_gyro_z(void)
{
    uint8_t buf[2];
    uint8_t reg = 0x47;
    i2c_master_write_read_device(
        I2C_NUM, MPU_ADDR, &reg, 1, buf, 2, pdMS_TO_TICKS(50));
    return (buf[0] << 8) | buf[1];
}

void gyro_calibrate(void)
{
    float sum = 0;
    for (int i = 0; i < 500; i++) {
        sum += -(mpu_read_gyro_z() / 65.5f) * M_PI / 180.0f;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    gyro_bias_z = sum / 500.0f;
}

// ======================================================
// UTIL
// ======================================================
static inline float wheel_speed(int dp)
{
    return 2 * M_PI * WHEEL_RADIUS * dp / (PULSE_PER_REV * DT);
}

static inline float ramp(float target, float current)
{
    if (target > current + DV_MAX) return current + DV_MAX;
    if (target < current - DV_MAX) return current - DV_MAX;
    return target;
}

static inline int pi_update(pi_t *p, float ref, float fb)
{
    float err = ref - fb;
    p->i += err * DT;
    if (p->i > 200) p->i = 200;
    if (p->i < 0)   p->i = 0;
    return (int)(p->kp * err + p->ki * p->i);
}

// ======================================================
// SPEED TASK
// ======================================================
void speed_task(void *arg)
{
    int16_t pl = 0, pr = 0;

    while (1) {
        if ((esp_timer_get_time() - last_cmd_time) > 200000) {
            vL_ref = 0;
            vR_ref = 0;
        }

        int16_t cL, cR;
        pcnt_get_counter_value(PCNT_L, &cL);
        pcnt_get_counter_value(PCNT_R, &cR);

        int dL = cL - pl;
        int dR = cR - pr;
        pl = cL; pr = cR;

        float vL = wheel_speed(dL);
        float vR = wheel_speed(dR);

        vL_cmd = ramp(vL_ref, vL_cmd);
        vR_cmd = ramp(vR_ref, vR_cmd);

        if (fabsf(vL_cmd) < 0.005f) { pwmL = 0; piL.i = 0; }
        if (fabsf(vR_cmd) < 0.005f) { pwmR = 0; piR.i = 0; }

        pwmL = pi_update(&piL, fabsf(vL_cmd), fabsf(vL));
        pwmR = pi_update(&piR, fabsf(vR_cmd), fabsf(vR));

        if (fabsf(vL_cmd) > 0.01f && pwmL < PWM_MIN_START) pwmL = PWM_MIN_START;
        if (fabsf(vR_cmd) > 0.01f && pwmR < PWM_MIN_START) pwmR = PWM_MIN_START;

        motor_set(pwmL, pwmR,
                  (vL_cmd >= 0) ? 1 : -1,
                  (vR_cmd >= 0) ? 1 : -1);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ======================================================
// ODOM + IMU FUSION TASK
// ======================================================
void odom_task(void *arg)
{
    while (1) {
        int16_t cL, cR;
        pcnt_get_counter_value(PCNT_L, &cL);
        pcnt_get_counter_value(PCNT_R, &cR);

        int dL = cL - lastL;
        int dR = cR - lastR;   // ROS forward x+
        lastL = cL;
        lastR = cR;

        float dl = 2 * M_PI * WHEEL_RADIUS * dL / PULSE_PER_REV;
        float dr = 2 * M_PI * WHEEL_RADIUS * dR / PULSE_PER_REV;

        float ds = (dl + dr) * 0.5f;
        float dth_enc = (dr - dl) / WHEEL_BASE;

        float gz = -(mpu_read_gyro_z() / 65.5f) * M_PI / 180.0f - gyro_bias_z;
        gyro_z_lpf = GYRO_ALPHA * gyro_z_lpf + (1 - GYRO_ALPHA) * gz;

        float dth;
        if (fabsf(ds) < 0.001f) {
            dth = dth_enc;
        } else if (fabsf(dth_enc) < 0.001f) {
            dth = gyro_z_lpf * DT;
        } else {
            dth = 0.7f * gyro_z_lpf * DT + 0.3f * dth_enc;
        }

        odom.th += dth;
        odom.x  += ds * cosf(odom.th);
        odom.y  += ds * sinf(odom.th);
        float th_deg = odom.th * 180.0f / M_PI;
        ESP_LOGI("ODOM",
         "x=%.3f  y=%.3f  th=%.3f deg",
         odom.x, odom.y, th_deg);

        char msg[128];
        snprintf(msg, sizeof(msg),
            "{\"odom_x\":%.4f,\"odom_y\":%.4f,\"odom_theta\":%.4f}\n",
            odom.x, odom.y, odom.th);
        
        uart_write_bytes(UART_NUM, msg, strlen(msg));

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ======================================================
// UART RX (/cmd_vel)
// ======================================================
void uart_task(void *arg)
{
    uint8_t buf[BUF_SIZE];

    while (1) {
        int len = uart_read_bytes(UART_NUM, buf, BUF_SIZE - 1, 100);
        if (len <= 0) continue;
        buf[len] = 0;

        float v = 0, w = 0;
        char *p;

        if ((p = strstr((char *)buf, "\"linear\"")))
            v = atof(p + 9);

        if ((p = strstr((char *)buf, "\"angular\"")))
            w = atof(p + 10);

        // MAP TO MOTOR REF (forward x+)
        v = v;      // no sign change
        w = -w;     // invert angular z to match ROS yaw

        if (v > MAX_V) v = MAX_V;
        if (v < -MAX_V) v = -MAX_V;
        if (w > MAX_W) w = MAX_W;
        if (w < -MAX_W) w = -MAX_W;

        vL_ref = v - w * WHEEL_BASE * 0.5f;
        vR_ref = v + w * WHEEL_BASE * 0.5f;

        last_cmd_time = esp_timer_get_time();
    }
}

// ======================================================
// MAIN
// ======================================================
void app_main(void)
{
    motor_init();
    encoder_init();
    i2c_init();
    mpu6050_init();
    gyro_calibrate();
    vacuum_init();

    uart_config_t u = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &u);
    uart_set_pin(UART_NUM, TX_PIN, RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    xTaskCreate(speed_task, "speed", 4096, NULL, 6, NULL);
    xTaskCreate(odom_task,  "odom",  4096, NULL, 5, NULL);
    xTaskCreate(uart_task,  "uart",  4096, NULL, 4, NULL);
    xTaskCreate(vacuum_task, "vacuum", 2048, NULL, 3, NULL);

    ESP_LOGI(TAG, "🚀 SLAM-READY FIRMWARE RUNNING");
}
