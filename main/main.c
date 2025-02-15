/*****************************************************************************
*                                                                           *
*  Copyright 2018 Simon M. Werner                                           *
*                                                                           *
*  Licensed under the Apache License, Version 2.0 (the "License");          *
*  you may not use this file except in compliance with the License.         *
*  You may obtain a copy of the License at                                  *
*                                                                           *
*    http://www.apache.org/licenses/LICENSE-2.0                             *
*                                                                           *
*  Unless required by applicable law or agreed to in writing, software      *
*  distributed under the License is distributed on an "AS IS" BASIS,        *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
*  See the License for the specific language governing permissions and      *
*  limitations under the License.                                           *
*                                                                           *
*  Modifications made by Gabriel Rodriguez (2024-2025)                      *
*****************************************************************************/

// Base Imports
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"

// For the MPU
#include "driver/i2c.h"
#include "ahrs.h"
#include "mpu9250.h"
#include "calibrate.h"
#include "common.h"

// For micro-ROS
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <sensor_msgs/msg/imu.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// For motor control
#include "driver/ledc.h"
#include "driver/gpio.h"
#define START_PIN   17
#define DIR_PIN     4
#define PWM_PIN     25
#define PWM_CHANNEL LEDC_CHANNEL_1
#define PWM_TIMER   LEDC_TIMER_0
#define PWM_MODE    LEDC_HIGH_SPEED_MODE
#define PWM_FREQ    20000  // 20 kHz
#define PWM_RES     LEDC_TIMER_8_BIT  // 8-bit resolution

// Setting additional parameters
static const char *TAG = "main";
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define CONFIG_MICRO_ROS_APP_STACK 4096
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 5
#define CONFIG_AHRS_APP_STACK 4096
#define CONFIG_CALIBRATION_APP_STACK 4096
#define CONFIG_MAX_APP_TASK_PRIO 30

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */

// Publisher and Message Type
rcl_publisher_t publisher;
sensor_msgs__msg__Imu msg;

// Define task handles
TaskHandle_t ahrsTaskHandle = NULL;
TaskHandle_t rosTaskHandle = NULL;
TaskHandle_t calibrationTaskHandle = NULL;

// Calibration constants
calibration_t cal = {

// magnetometer offset from calibration
.mag_offset = {.x = 29.093750, .y = 50.800781, .z = -68.906250},
.mag_scale = {.x = 0.988142, .y = 0.999372, .z = 1.012790},

// accelerometer offsets from calibration
.accel_offset = {.x = -0.010017, .y = 0.056238, .z = -0.372841},
.accel_scale_lo = {.x = 0.995519, .y = 1.031251, .z = 0.828868},
.accel_scale_hi = {.x = -0.998890, .y = -0.966249, .z = -1.204864},

// gyroscope bias from calibration, averaged, from 01/18
.gyro_bias_offset = {.x = -3.210659, .y = 2.017682, .z = -0.724418}
};

/**
* Transformation for accelerometer and gyroscope:
*  - Rotate around Z axis 180 degrees
*  - Rotate around X axis -90 degrees
* @param  {object} s {x,y,z} sensor
* @return {object}   {x,y,z} transformed
*/
static void transform_accel_gyro(vector_t *v)
{
 float x = v->x;
 float y = v->y;
 float z = v->z;
 

 v->x = -x;
 v->y = -z;
 v->z = -y;
}

/**
* Transformation: to get magnetometer aligned
* @param  {object} s {x,y,z} sensor
* @return {object}   {x,y,z} transformed
*/
static void transform_mag(vector_t *v)
{
 float x = v->x;
 float y = v->y;
 float z = v->z;


 v->x = -y;
 v->y = z;
 v->z = -x;
}

// Shared variables
static double q_w, q_x, q_y, q_z;
vector_t va, vg, vm;

// Mutex to protect the shared data
static SemaphoreHandle_t imu_message_mutex;

// Function to initialize motor control pins and PWM
void motor_init() {
  // Configure START pin
  gpio_pad_select_gpio(START_PIN);
  gpio_set_direction(START_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(START_PIN, 1); // Start HIGH

  // Configure DIR pin
  gpio_pad_select_gpio(DIR_PIN);
  gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(DIR_PIN, 0); // Default forward

  // Configure LEDC PWM
  ledc_timer_config_t timer_conf = {
      .duty_resolution = PWM_RES,
      .freq_hz = PWM_FREQ,
      .speed_mode = PWM_MODE,
      .timer_num = PWM_TIMER
  };
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t ledc_conf = {
      .channel    = PWM_CHANNEL,
      .duty       = 0, // Start with motor off
      .gpio_num   = PWM_PIN,
      .speed_mode = PWM_MODE,
      .hpoint     = 0,
      .timer_sel  = PWM_TIMER
  };
  ledc_channel_config(&ledc_conf);
}

// Function to set motor speed
void motor_control(int speed) {
  if (speed > 0) {
      gpio_set_level(DIR_PIN, 0); // Forward
  } else if (speed < 0) {
      gpio_set_level(DIR_PIN, 1); // Reverse
  }

  // Convert speed to 8-bit PWM value (0-255)
  uint32_t duty = 255 - abs(speed);
  ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty);
  ledc_update_duty(PWM_MODE, PWM_CHANNEL);
}

// Task to run the AHRS algorithm as fast as possible.
static void ahrs_task(void *arg)
{
 // Initialize MPU with calibration (cal defined above) and algorithm frequency
 i2c_mpu9250_init(&cal);
 ahrs_init(SAMPLE_FREQ_Hz, 0.8);
 memset(&msg, 0, sizeof(msg)); 

 while (true)
 {
    // Get the Accelerometer, Gyroscope and Magnetometer values.
    ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));

    // Transform these values to the orientation of our device.
    transform_accel_gyro(&va);
    transform_accel_gyro(&vg);
    transform_mag(&vm);

    // Apply the AHRS algorithm
    ahrs_update(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
                va.x, va.y, va.z,
                vm.x, vm.y, vm.z);

    // Getting the quaternion from the algorithm
    float w, x, y, z;
    ahrs_get_quaternion(&w, &x, &y, &z);

    // Assinging the result to the shared doubles.
    // Lock the mutex before updating shared data
    if (xSemaphoreTake(imu_message_mutex, portMAX_DELAY))
    {
      q_w = w;
      q_x = x;
      q_y = y;
      q_z = z;
      xSemaphoreGive(imu_message_mutex);
    }

    // Example motor control logic based on pitch (y-axis rotation)
    if (q_y > 0.1) 
    {
      motor_control(100);  // Move forward
    } else if (q_y < -0.1) {
      motor_control(-100); // Move backward
    } else {
      motor_control(0);    // Stop
    }
    
    taskYIELD();
 }
}

// Function to publish IMU messages
void imu_callback(void)
{
    if (xSemaphoreTake(imu_message_mutex, portMAX_DELAY))
    {
        // Assign Quaternion to message
        msg.orientation.w = q_w;
        msg.orientation.x = q_x;
        msg.orientation.y = q_y;
        msg.orientation.z = q_z;

        // Assign Angular Velocities to message
        msg.angular_velocity.x = DEG2RAD(vg.x);
        msg.angular_velocity.y = DEG2RAD(vg.y);
        msg.angular_velocity.z = DEG2RAD(vg.z);

        // Assign Linear Accelerations to message
        msg.linear_acceleration.x = va.x;
        msg.linear_acceleration.y = va.y;
        msg.linear_acceleration.z = va.z;

        xSemaphoreGive(imu_message_mutex);
    }

    // Assign covariances as unknowns (they are not needed)
    memset(msg.orientation_covariance, 0, sizeof(msg.orientation_covariance));
    memset(msg.angular_velocity_covariance, 0, sizeof(msg.angular_velocity_covariance));
    memset(msg.linear_acceleration_covariance, 0, sizeof(msg.linear_acceleration_covariance));

    // Publish the message and check for errors
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    if (ret == RCL_RET_OK) {
        ESP_LOGI(TAG, "IMU Message published successfully");
    } else {
        ESP_LOGI(TAG, "Failed to publish IMU Message");
    }
}

// Task Function for Micro-ROS and Calibration
static void micro_ros_task(void *arg)
{
  const TickType_t xFrequency = pdMS_TO_TICKS(20);  // 50 Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();

  ESP_LOGI(TAG, "Starting micro_ros_task...");
    
  //still want currently frequency for future control
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));

    #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
      rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
      // Static Agent IP and port can be used instead of autodisvery.
      RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
      //RCCHECK(rmw_uros_discover_agent(rmw_options));
    #endif
  
    // create init_options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_imu_publisher", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu_data"));

    ESP_LOGI(TAG, "micro_ros_task initialized. Entering loop...");

    while(1){

      ESP_LOGI(TAG, "Calling imu_callback()");
      imu_callback();

      ESP_LOGI(TAG, "imu_callback() finished. Waiting for next cycle...");
      vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Wait 20ms for next cycle
    }

  // Exit and clean up
  RCCHECK(rcl_publisher_fini(&publisher, &node));
  RCCHECK(rcl_node_fini(&node));
  i2c_driver_delete(I2C_MASTER_NUM);
  vTaskDelete(NULL); 
}

// Task to calbiration the MPU9250
static void calibration_task(void *arg)
{
  ESP_LOGI(TAG, "Calibration mode is enabled. Running calibration...");

  vector_t vg_sum_sum = {0}, offset_holder_sum = {0}, scale_lo_holder_sum = {0}, scale_hi_holder_sum = {0};
  vector_t v_min_holder_sum = {0}, v_max_holder_sum = {0}, v_scale_holder_sum = {0};

  for (int i = 0; i < 3; i++) 
  {
    ESP_LOGI(TAG,"Calibration iteration %d", i + 1);

    vector_t vg_sum_holder, offset_holder, scale_lo_holder, scale_hi_holder;
    vector_t v_min_holder, v_max_holder, v_scale_holder;

    calibrate_gyro(&vg_sum_holder);
    calibrate_accel(&offset_holder, &scale_lo_holder, &scale_hi_holder);
    calibrate_mag(&v_min_holder, &v_max_holder, &v_scale_holder);

    // AGGREGATING THE CALIBRATION VALUES
    vg_sum_sum.x += vg_sum_holder.x;
    vg_sum_sum.y += vg_sum_holder.y;
    vg_sum_sum.z += vg_sum_holder.z;
    offset_holder_sum.x += offset_holder.x;
    offset_holder_sum.y += offset_holder.y;
    offset_holder_sum.z += offset_holder.z;
    scale_lo_holder_sum.x += scale_lo_holder.x;
    scale_lo_holder_sum.y += scale_lo_holder.y;
    scale_lo_holder_sum.z += scale_lo_holder.z;
    scale_hi_holder_sum.x += scale_hi_holder.x;
    scale_hi_holder_sum.y += scale_hi_holder.y;
    scale_hi_holder_sum.z += scale_hi_holder.z;
    v_min_holder_sum.x += v_min_holder.x;
    v_min_holder_sum.y += v_min_holder.y;
    v_min_holder_sum.z += v_min_holder.z;
    v_max_holder_sum.x += v_max_holder.x;
    v_max_holder_sum.y += v_max_holder.y;
    v_max_holder_sum.z += v_max_holder.x;
    v_scale_holder_sum.x += v_scale_holder.x;
    v_scale_holder_sum.y += v_scale_holder.y;
    v_scale_holder_sum.z += v_scale_holder.z;

    if (i != 2) 
    {
      ESP_LOGI(TAG,"Calibration iteration %d completed, wait three seconds for the next.", i + 1);
    }
    else
    {
      ESP_LOGI(TAG,"Calibration iteration %d completed. The average of the 3 calibrations will be provided.", i + 1);
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
  }

  // AVERAGING THE CALIBRATION VALUES
  vector_t vg_sum_avg, offset_avg, scale_lo_avg, scale_hi_avg;
  vector_t v_min_avg, v_max_avg, v_scale_avg;
  vg_sum_avg.x = vg_sum_sum.x/3.0;
  vg_sum_avg.y = vg_sum_sum.y/3.0;
  vg_sum_avg.z = vg_sum_sum.z/3.0;
  offset_avg.x = offset_holder_sum.x/3.0;
  offset_avg.y = offset_holder_sum.y/3.0;
  offset_avg.z = offset_holder_sum.z/3.0;
  scale_lo_avg.x = scale_lo_holder_sum.x/3.0;
  scale_lo_avg.y = scale_lo_holder_sum.y/3.0;
  scale_lo_avg.z = scale_lo_holder_sum.z/3.0;
  scale_hi_avg.x = scale_hi_holder_sum.x/3.0;
  scale_hi_avg.y = scale_hi_holder_sum.y/3.0;
  scale_hi_avg.z = scale_hi_holder_sum.z/3.0;
  v_min_avg.x = v_min_holder_sum.x/3.0;
  v_min_avg.y = v_min_holder_sum.y/3.0;
  v_min_avg.z = v_min_holder_sum.z/3.0;
  v_max_avg.x = v_max_holder_sum.x/3.0;
  v_max_avg.y = v_max_holder_sum.y/3.0;
  v_max_avg.z = v_max_holder_sum.z/3.0;
  v_scale_avg.x = v_scale_holder_sum.x/3.0;
  v_scale_avg.y = v_scale_holder_sum.y/3.0;
  v_scale_avg.z = v_scale_holder_sum.z/3.0;

  // Print the results:
  ESP_LOGI(TAG,"    .mag_offset = {.x = %f, .y = %f, .z = %f}", (v_min_avg.x + v_max_avg.x) / 2, (v_min_avg.y + v_max_avg.y) / 2, (v_min_avg.z + v_max_avg.z) / 2);
  ESP_LOGI(TAG,"    .mag_scale = {.x = %f, .y = %f, .z = %f}", v_scale_avg.x, v_scale_avg.y, v_scale_avg.z);
  ESP_LOGI(TAG,"    .accel_offset = {.x = %f, .y = %f, .z = %f}", offset_avg.x, offset_avg.y, offset_avg.z);
  ESP_LOGI(TAG,"    .accel_scale_lo = {.x = %f, .y = %f, .z = %f}", scale_lo_avg.x, scale_lo_avg.y, scale_lo_avg.z);
  ESP_LOGI(TAG,"    .accel_scale_hi = {.x = %f, .y = %f, .z = %f}", scale_hi_avg.x, scale_hi_avg.y, scale_hi_avg.z);
  ESP_LOGI(TAG,"    .gyro_bias_offset = {.x = %f, .y = %f, .z = %f}", vg_sum_avg.x, vg_sum_avg.y, vg_sum_avg.z);

  i2c_driver_delete(I2C_MASTER_NUM);
  vTaskDelete(NULL);
}

void app_main(void)
{
  // If in calibration mode...
  #if defined(CONFIG_CALIBRATION_MODE)
    // Calibration Task
    xTaskCreatePinnedToCore(
      calibration_task,
      "calibration_task",
      CONFIG_CALIBRATION_APP_STACK,
      NULL,
      CONFIG_MAX_APP_TASK_PRIO-1,
      &calibrationTaskHandle,
      tskNO_AFFINITY);

  // If not in calibration mode...
  #else
    // Initialize the mutex
    imu_message_mutex = xSemaphoreCreateMutex();
    if (imu_message_mutex == NULL) {
      ESP_LOGE(TAG, "Failed to create mutex!");
      return;
    }

    // Set up the wifi connection
    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
      ESP_ERROR_CHECK(uros_network_interface_initialize());
    #endif
    
    // AHRS Task
    xTaskCreatePinnedToCore(
      ahrs_task,
      "ahrs_task",
      CONFIG_AHRS_APP_STACK,
      NULL,
      CONFIG_MAX_APP_TASK_PRIO-1,
      &ahrsTaskHandle,
      tskNO_AFFINITY);

    // Micro ROS Task
    xTaskCreatePinnedToCore(
      micro_ros_task,
      "micro_ros_task",
      CONFIG_MICRO_ROS_APP_STACK,
      NULL,
      CONFIG_MICRO_ROS_APP_TASK_PRIO,
      &rosTaskHandle,
      tskNO_AFFINITY);
  #endif
}