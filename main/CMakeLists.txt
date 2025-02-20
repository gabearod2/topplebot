idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES ahrs mpu9250 micro_ros_espidf_component motor_control)
