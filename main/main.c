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


//static const char *TAG = "main";


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


// Structure to hold quaternion data
typedef struct {
 double x;
 double y;
 double z;
 double w;
} quaternion;


calibration_t cal = {
 // magnetometer offset from calibration
 .mag_offset = {.x = -22.562500, .y = 40.042969, .z = -78.667969},
 .mag_scale = {.x = 1.027731, .y = 1.012845, .z = 0.961848},


 // accelerometer offsets from calibration
 .accel_offset = {.x = -0.011785, .y = 0.039968, .z = -0.404508},
 .accel_scale_lo = {.x = 0.995519, .y = 1.031251, .z = 0.828868},
 .accel_scale_hi = {.x = -0.998890, .y = -0.966249, .z = -1.204864},


 // gyroscope bias from calibration
 .gyro_bias_offset = {.x = -3.247107, .y = 1.918163, .z = -0.634698}
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


/**
* Conversion: to get quaternion from Euler angles
* @param  {object} s {x,y,z} sensor
* @return {object}   {x,y,z} transformed
*/
static void get_quaternion(quaternion *q, float *heading, float *pitch, float *roll)
{
 // Cast float values to double
   double d_roll = (double)*roll;
   double d_pitch = (double)*pitch;
   double d_yaw = (double)*heading;


 // Abbreviations for the various angular functions
   double cr = cos(d_roll * 0.5);
   double sr = sin(d_roll * 0.5);
   double cp = cos(d_pitch * 0.5);
   double sp = sin(d_pitch * 0.5);
   double cy = cos(d_yaw * 0.5);
   double sy = sin(d_yaw * 0.5);




   // The quaternion data
   q->x = sr * cp * cy - cr * sp * sy;
   q->y = cr * sp * cy + sr * cp * sy;
   q->z = cr * cp * sy - sr * sp * cy;
 q->w = cr * cp * cy + sr * sp * sy;
}






void imu_callback(void)
{
 // Initialize MPU with calibration (cal defined above) and algorithm frequency
 i2c_mpu9250_init(&cal);
 ahrs_init(SAMPLE_FREQ_Hz, 0.8);


 memset(&msg, 0, sizeof(msg)); 


 //uint64_t i = 0;
 while (true)
 {
 vector_t va, vg, vm; // initializing the three vectors


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
   float heading_rad, pitch_rad, roll_rad;
   ahrs_get_euler_in_degrees(&heading, &pitch, &roll);
   heading_rad = DEG2RAD(heading);
   pitch_rad = DEG2RAD(pitch);
   roll_rad = DEG2RAD(roll);




   // Normal printing log
     //ESP_LOGI(TAG, "Roll: %2.3f°, Pitch: %2.3f°, Yaw/Heading: %2.3f°,Temp %2.3f°C", roll, pitch, heading, temp);
     //ESP_LOGI(TAG, "Acceleration, x: %2.3f m/s, Acceleration, y: %2.3f m/s, Acceleration z: %2.3f m/s", va.x, va.y, va.z);
     //ESP_LOGI(TAG, "Roll Rate: %2.3f°/s, Pitch Rate: %2.3f°/s, Yaw Rate: %2.3f°/s", vg.x, vg.y, vg.z);
     //ESP_LOGI(TAG, "The Quaternion: w: %2.3f, x: %2.3f, y: %2.3f, z: %2.3f", q.w, q.x, q.y, q.z);


     // Make the WDT happy
     vTaskDelay(0);


   // Publish to ROS the data every 30 items
   // Getting the quaternion
   quaternion q;
   get_quaternion(&q, &heading_rad, &pitch_rad, &roll_rad);


     // Assign Quaternion to message
       msg.orientation.w = q.w;
       msg.orientation.x = q.x;
       msg.orientation.y = q.y;
       msg.orientation.z = q.z;


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
     // Publish the message if it is available


       printf("Publishing IMU Message to the imu_data Topic");
       RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
 }
}


void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
 RCLC_UNUSED(last_call_time);
 if (timer != NULL) {
   imu_callback();
   printf("Called imu callback");


 }
}


static void imu_micro_ros_task(void *arg)
{
// If in calibration mode, only calibrate
#ifdef CONFIG_CALIBRATION_MODE
 calibrate_gyro();
 calibrate_accel();
 calibrate_mag();
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
 const unsigned int timer_timeout = 20; //20 ms timer for publishing
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


   // Exit
   vTaskDelay(100 / portTICK_PERIOD_MS);
   i2c_driver_delete(I2C_MASTER_NUM);
   vTaskDelete(NULL);
 #endif
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