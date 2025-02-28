#include "motor_control.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

// Motor 1 Config
#define START_PIN_1   5
#define DIR_PIN_1     23
#define PWM_PIN_1     25
#define PWM_CHANNEL_1 LEDC_CHANNEL_1

// Motor 2 Config
#define START_PIN_2   17
#define DIR_PIN_2     22
#define PWM_PIN_2     26
#define PWM_CHANNEL_2 LEDC_CHANNEL_2

// Motor 3 Config
#define START_PIN_3   16
#define DIR_PIN_3     21
#define PWM_PIN_3     27
#define PWM_CHANNEL_3 LEDC_CHANNEL_3

// Other Settings
#define PWM_TIMER   LEDC_TIMER_0
#define PWM_MODE    LEDC_HIGH_SPEED_MODE
#define PWM_FREQ    20000  // 20 kHz
#define PWM_RES     LEDC_TIMER_8_BIT  // 8-bit resolution

// Function to Initialize Motor Control Pins and PWMs
void motor_init() {
    // Configuring START Pins
    esp_rom_gpio_pad_select_gpio(START_PIN_1);
    esp_rom_gpio_pad_select_gpio(START_PIN_2);
    esp_rom_gpio_pad_select_gpio(START_PIN_3);
    gpio_set_direction(START_PIN_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(START_PIN_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(START_PIN_3, GPIO_MODE_OUTPUT);
    gpio_set_level(START_PIN_1, 1);
    gpio_set_level(START_PIN_2, 1);
    gpio_set_level(START_PIN_3, 1);
  
    // Configuring DIRECTION Pins  
    esp_rom_gpio_pad_select_gpio(DIR_PIN_1);
    esp_rom_gpio_pad_select_gpio(DIR_PIN_2);
    esp_rom_gpio_pad_select_gpio(DIR_PIN_3);
    gpio_set_direction(DIR_PIN_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(DIR_PIN_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(DIR_PIN_3, GPIO_MODE_OUTPUT);
    gpio_set_level(DIR_PIN_1, 0);
    gpio_set_level(DIR_PIN_2, 0);
    gpio_set_level(DIR_PIN_3, 0);
  
    // Configure PWM Timer
    ledc_timer_config_t timer_conf = {
        .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ,
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER
    };
    ledc_timer_config(&timer_conf);
  
    // Configure PWM for each motor
    ledc_channel_config_t ledc_conf_1 = {
        .channel    = PWM_CHANNEL_1,
        .duty       = 0,
        .gpio_num   = PWM_PIN_1,
        .speed_mode = PWM_MODE,
        .hpoint     = 0,
        .timer_sel  = PWM_TIMER
    };
    ledc_channel_config(&ledc_conf_1);

    ledc_channel_config_t ledc_conf_2 = {
        .channel    = PWM_CHANNEL_2,
        .duty       = 0,
        .gpio_num   = PWM_PIN_2,
        .speed_mode = PWM_MODE,
        .hpoint     = 0,
        .timer_sel  = PWM_TIMER
    };
    ledc_channel_config(&ledc_conf_2);

    ledc_channel_config_t ledc_conf_3 = {
        .channel    = PWM_CHANNEL_3,
        .duty       = 0,
        .gpio_num   = PWM_PIN_3,
        .speed_mode = PWM_MODE,
        .hpoint     = 0,
        .timer_sel  = PWM_TIMER
    };
    ledc_channel_config(&ledc_conf_3);
}

// Function to Set Motor 1 Speed
void motor_control_1(int speed, bool type) {
    // Setting the start pin
    if (speed < 10 || speed > 10) {
        gpio_set_level(START_PIN_1, 0); // Motor On
    } else {
        gpio_set_level(START_PIN_1, 1); // Motor Off, Brake On
    }

    // Setting the direction pin
    if (speed > 0) {
        gpio_set_level(DIR_PIN_1, 0); // Forward
    } else if (speed < 0) {
        gpio_set_level(DIR_PIN_1, 1); // Reverse
    }

    uint32_t duty = abs(speed); // 0 to 255
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_1, duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_1);
}

// Function to Set Motor 2 Speed
void motor_control_2(int speed, bool type) {
    // Setting the start pin
    if (speed < 10 || speed > 10) {
        gpio_set_level(START_PIN_2, 0); // Motor On
    } else {
        gpio_set_level(START_PIN_2, 1); // Motor Off, Brake On
    }

    // Setting the direction pin
    if (speed > 0) {
        gpio_set_level(DIR_PIN_2, 0); // Forward
    } else if (speed < 0) {
        gpio_set_level(DIR_PIN_2, 1); // Reverse
    }

    uint32_t duty = abs(speed);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_2, duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_2);
}

// Function to Set Motor 3 Speed
void motor_control_3(int speed, bool type) {
    // Setting the start pin
    if (speed < 10 || speed > 10) {
        gpio_set_level(START_PIN_3, 0); // Motor On
    } else {
        gpio_set_level(START_PIN_3, 1); // Motor Off, Brake On
    }

    // Setting the direction pin
    if (speed > 0) {
        gpio_set_level(DIR_PIN_3, 0); // Forward
    } else if (speed < 0) {
        gpio_set_level(DIR_PIN_3, 1); // Reverse
    }

    uint32_t duty = abs(speed);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_3, duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_3);
}
