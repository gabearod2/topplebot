/*****************************************************************************
*                                                                        *
*  Copyright 2018 Simon M. Werner                                        *
*                                                                        *
*  Licensed under the Apache License, Version 2.0 (the "License");       *
*  you may not use this file except in compliance with the License.      *
*  You may obtain a copy of the License at                               *
*                                                                        *
*    http://www.apache.org/licenses/LICENSE-2.0                          *
*                                                                        *
*  Unless required by applicable law or agreed to in writing, software   *
*  distributed under the License is distributed on an "AS IS" BASIS,     *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
*  See the License for the specific language governing permissions and   *
*  limitations under the License.                                        *
*                                                                        *
*  Modifications made by Gabriel Rodriguez                               *
*****************************************************************************/

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


// static const char *TAG = "main";

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define CONFIG_MICRO_ROS_APP_STACK 4000
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 5

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */

rcl_publisher_t publisher;
sensor_msgs__msg__Imu msg;

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

// Function to publish the current data from the IMU
void imu_callback(void)
{
 // Initialize MPU with calibration (cal defined above) and algorithm frequency
 i2c_mpu9250_init(&cal);
 ahrs_init(SAMPLE_FREQ_Hz, 0.8);
 memset(&msg, 0, sizeof(msg)); 

 while (true)
 {
    // Initializing the three vectors for sensor data
    vector_t va, vg, vm; 
    
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

    // Getting temp
    float temp;
    ESP_ERROR_CHECK(get_temperature_celsius(&temp));
  
    // Getting heading pitch and roll
    float heading, pitch, roll;
    ahrs_get_euler_in_degrees(&heading, &pitch, &roll);
  
    // Normal printing log
    //ESP_LOGI(TAG, "Roll: %2.3f°, Pitch: %2.3f°, Yaw/Heading: %2.3f°,Temp %2.3f°C", roll, pitch, heading, temp);
    //ESP_LOGI(TAG, "Acceleration, x: %2.3f m/s, Acceleration, y: %2.3f m/s, Acceleration z: %2.3f m/s", va.x, va.y, va.z);
    //ESP_LOGI(TAG, "Roll Rate: %2.3f°/s, Pitch Rate: %2.3f°/s, Yaw Rate: %2.3f°/s", vg.x, vg.y, vg.z);
    //ESP_LOGI(TAG, "The Quaternion: w: %2.3f, x: %2.3f, y: %2.3f, z: %2.3f", q.w, q.x, q.y, q.z);

    // Make the WDT happy
    vTaskDelay(0);

    // Getting the quaternion
    float w, x, y, z;
    ahrs_get_quaternion(&w, &x, &y, &z);

    // If you need double precision
    double w_d = w;
    double x_d = x;
    double y_d = y;
    double z_d = z;

    // Assign Quaternion to message
    msg.orientation.w = w_d;
    msg.orientation.x = x_d;
    msg.orientation.y = y_d;
    msg.orientation.z = z_d;

    // Assign Angular Velocities to message
    msg.angular_velocity.x = DEG2RAD(vg.x);
    msg.angular_velocity.y = DEG2RAD(vg.y);
    msg.angular_velocity.z = DEG2RAD(vg.z);
    
    // Assign Linear Accelerations to message
    msg.linear_acceleration.x = va.x;
    msg.linear_acceleration.y = va.y;
    msg.linear_acceleration.z = va.z;

    // Assign covariances as unkowns (they are not needed) to message
    memset(msg.orientation_covariance, 0, sizeof(msg.orientation_covariance));
    memset(msg.angular_velocity_covariance, 0, sizeof(msg.angular_velocity_covariance));
    memset(msg.linear_acceleration_covariance, 0, sizeof(msg.linear_acceleration_covariance));
    
    // Publish the resulting ros message
    printf("Publishing IMU Message to the imu_data Topic");
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
 }
}

// Timer to call the imu_callback on set intervals
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
 RCLC_UNUSED(last_call_time);
 if (timer != NULL) 
 {
    imu_callback();
    printf("Called imu callback");
 }
}

// Task Function
static void imu_micro_ros_task(void *arg)
{
  // If in calibration mode, only calibrate
  #ifdef CONFIG_CALIBRATION_MODE

  for (int i = 0; i < 3; i++) 
  {
    ESP_LOGI("calibration","Calibration iteration %d", i + 1);

    calibrate_gyro();
    calibrate_accel();
    calibrate_mag();

    ESP_LOGI("calibration","Calibration iteration %d completed, wait three seconds for the next.", i + 1);
    vTaskDelay(pdMS_TO_TICKS(3000));

  }

  #else
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


    // create timer,
    rcl_timer_t timer;
    const unsigned int timer_timeout = 5; //5 ms timer for publishing
    RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    while(1){
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      //usleep(10000);
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_node_fini(&node));
  #endif
  
  // Exit and clean up
  vTaskDelay(100 / portTICK_PERIOD_MS);
  i2c_driver_delete(I2C_MASTER_NUM);
  vTaskDelete(NULL); 
}


void app_main(void)
{
  #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
  #endif
 
 // start i2c task
 xTaskCreate(imu_micro_ros_task,
     "imu_micro_ros_task",
     CONFIG_MICRO_ROS_APP_STACK,
     NULL,
     CONFIG_MICRO_ROS_APP_TASK_PRIO,
     NULL);
}