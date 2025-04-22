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

// For the MPU 9250 and Motors
#include "driver/i2c.h"
#include "ahrs.h"
#include "mpu9250.h"
#include "calibrate.h"
#include "common.h"
#include "motor_control.h"

// For micro-ROS
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <sensor_msgs/msg/imu.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Setting additional parameters
static const char *TAG = "main";
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

// Task Config
#define CONFIG_MICRO_ROS_APP_STACK 4096
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 5
#define CONFIG_AHRS_APP_STACK 4096
#define CONFIG_CALIBRATION_APP_STACK 4096
#define CONFIG_MAX_APP_TASK_PRIO 30

// More Micro-ROS and I2C Config
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */

// Publisher and Message Type
rcl_publisher_t publisher;
sensor_msgs__msg__Imu msg;

// Defining the Task Handles
TaskHandle_t ahrsTaskHandle = NULL;
TaskHandle_t rosTaskHandle = NULL;
TaskHandle_t calibrationTaskHandle = NULL;

// Calibration Constants for IMU
calibration_t cal = {

// magnetometer offset from calibration
.mag_offset = {.x = 17.244791, .y = 40.773438, .z = 7.773436},
.mag_scale = {.x = 0.999196, .y = 1.010870, .z = 0.993069},

// accelerometer offsets from calibration
.accel_offset = {.x = -0.102838, .y = 0.007450, .z = -0.093568},
.accel_scale_lo = {.x = 0.952312, .y = 1.007089, .z = 0.961626},
.accel_scale_hi = {.x = -1.051078, .y = -0.994280, .z = -1.048755},

// gyroscope bias from calibration, averaged, from 01/18
.gyro_bias_offset = {.x = -2.731378, .y = 5.108589, .z = 0.935637}
};

struct quaternion {
  float w, x, y, z;
};

// Quaternion multiplication: q1 * q2 → q3
static void quaternion_multiply(const struct quaternion *q1, const struct quaternion *q2, struct quaternion *q3) {
  q3->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
  q3->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
  q3->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
  q3->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
}

// Quaternion inverse (conjugate for unit quaternions)
static void quaternion_inverse(struct quaternion *q) {
  q->x = -q->x;
  q->y = -q->y;
  q->z = -q->z;
}

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

// Initializing Shared Variables
static double q_w, q_x, q_y, q_z;
vector_t va, vg, vm;

// Initializing Mutex to Protect the Shared Data
static SemaphoreHandle_t imu_message_mutex;

// Task to run the AHRS algorithm as fast as possible.
static void ahrs_task(void *arg)
{
  // Initialize MPU with calibration (cal defined above) and algorithm frequency
  i2c_mpu9250_init(&cal);
  ahrs_init(SAMPLE_FREQ_Hz, 0.8);
  memset(&msg, 0, sizeof(msg)); 

  // Initializing variables before the loop:
  struct quaternion q_des, q_current, q_err;
  float roll_err, pitch_err, yaw_err;
  float roll_err_i, pitch_err_i, yaw_err_i;
  float raw_roll_rate, raw_pitch_rate, raw_yaw_rate;
  float roll_rate_err = 0.0f;
  float pitch_rate_err = 0.0f;
  float yaw_rate_err = 0.0f;
  int count = 0;
  float kp = 72.0;// 76.0
  float ki = 3.0;// 0.25
  float kd = 21.0;// 21.0
  float speed_roll = 0;
  roll_err_i = 0; 
  float speed_pitch = 0;
  pitch_err_i = 0;
  float speed_yaw = 0; 
  yaw_err_i = 0;
  q_des = (struct quaternion){0, 0, 0, 0};
  const float decay = 0.99f;

  float alpha = 0.15f;  // You can tune this (0.01 to 0.1 range is typical)
  
  // Loop to update AHRS and assign imu info to mutex
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

    // Getting the quaternion from the AHRS algorithm from NED
    ahrs_get_quaternion(&q_current.w, &q_current.x, &q_current.y, &q_current.z);

    // Getting raw gyroscope values
    raw_roll_rate = vg.x;
    raw_pitch_rate = -vg.y;
    raw_yaw_rate = -vg.z;

    // Applying LPF to each gyroscope reading
    roll_rate_err = alpha * raw_roll_rate + (1.0f - alpha) * roll_rate_err;
    pitch_rate_err = alpha * raw_pitch_rate + (1.0f - alpha) * pitch_rate_err;
    yaw_rate_err = alpha * raw_yaw_rate + (1.0f - alpha) * yaw_rate_err;

    /*
    Roll Control - Motor 1
    Pitch Contorl - Motor 3
    Yaw Control - Motor 2
    */

    // Assigning the result to the shared doubles, lock the mutex before updating shared data
    if (xSemaphoreTake(imu_message_mutex, 0))
    {
      q_w = q_current.w;
      q_x = q_current.x;
      q_y = q_current.y;
      q_z = q_current.z;
      xSemaphoreGive(imu_message_mutex);
    }

    //ESP_LOGI(TAG, "QUATERNION { X:%.3f Y: %.3f Z: %.3f W: %.3f }", q_x,q_y,q_z,q_w);

    if (count < 7500)
    {
      count += 1;
    }
    else if (count == 7500)
    {
      q_des.w = q_current.w;
      q_des.x = q_current.x;
      q_des.y = q_current.y;
      q_des.z = q_current.z;
      quaternion_inverse(&q_des);
      count += 1;
    }
    else
    {
      // Quaternion Error
      quaternion_multiply(&q_des, &q_current, &q_err);

      // Position Error Definition
      roll_err = 2*180*q_err.x/3.1415;
      pitch_err = -2*180*q_err.y/3.1415;
      yaw_err = -2*180*q_err.z/3.1415;

      // Emergency shutdown condition: if any angle exceeds ±20 degrees
      bool shutdown = fabsf(roll_err) > 20.0f || fabsf(pitch_err) > 20.0f || fabsf(yaw_err) > 20.0f;

      // Integral Error
      //if (fabsf(roll_err) > 1.0f || fabsf(pitch_err) >1.0f || fabsf(yaw_err) > 5.0f)
      //{

          roll_err_i = roll_err_i + roll_err;
        
          pitch_err_i = pitch_err_i + pitch_err;

          yaw_err_i = yaw_err_i + yaw_err;

      ///}
      //else
      //{
        //roll_err_i = 0;
        //pitch_err_i = 0;
        //yaw_err_i = 0;
      //}
      
      // Clamp the integral contributions
      roll_err_i = fmaxf(-255.0f / ki, fminf(roll_err_i, 255.0f / ki));
      pitch_err_i = fmaxf(-255.0f / ki, fminf(pitch_err_i, 255.0f / ki));
      yaw_err_i = fmaxf(-255.0f / ki, fminf(yaw_err_i, 255.0f / ki));

      // Determining control input
      speed_roll = kp*roll_err + kd*roll_rate_err + ki*roll_err_i;
      speed_pitch = kp*pitch_err + kd*pitch_rate_err + ki*pitch_err_i;
      speed_yaw = kp*yaw_err + kd*yaw_rate_err + ki*yaw_err_i;
      
      //if (count % 20 == 0)
      //{
        //ESP_LOGI(TAG, "Omega_x, Omega_y, Omega_z: {x = %.3f,y = %.3f,z = %.3f,}",vg.x,vg.y,vg.z);
        //ESP_LOGI(TAG, "Quaternion: {%.3f, %.3f, %.3f}", q_current.x, q_current.y, q_current.z);
      //}
      
      if (shutdown) {
        motor_control_1(0, false); // or false to brake if supported
        motor_control_2(0, false);
        motor_control_3(0, false);
        ESP_LOGW(TAG, "MOTORS SHUT DOWN: Orientation error exceeded safety threshold.");
        roll_err_i = 0;
      } else {
        // Clamp final command to valid PWM range
        speed_roll  = fmaxf(-255.0f, fminf(speed_roll, 255.0f));
        speed_pitch = fmaxf(-255.0f, fminf(speed_pitch, 255.0f));
        speed_yaw   = fmaxf(-255.0f, fminf(speed_yaw, 255.0f));
      
        // Send to motors
        motor_control_1((int) speed_roll, true);
        motor_control_2((int) speed_yaw, true);
        motor_control_3((int) speed_pitch, true);
      }      
      count += 1;
    }

    taskYIELD();
  }
}

// Task Function to publish IMU messages
void imu_callback(void)
{
  // Pulling the imu information from the mutex
  if (xSemaphoreTake(imu_message_mutex, 0))
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

  // Publish the message and check for errors
  rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);

  /* Debugging ...
  if (ret == RCL_RET_OK) {
    ESP_LOGI(TAG, "IMU Message published successfully");
  } else {
    ESP_LOGI(TAG, "Failed to publish IMU Message");
  }
  */
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

  // Looping to update the quaternion and publish
  while(1)
  {
    // ESP_LOGI(TAG, "Calling imu_callback()");
    imu_callback();
    // ESP_LOGI(TAG, "imu_callback() finished. Waiting for next cycle...");
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

    // Initialize the motor
    motor_init();

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