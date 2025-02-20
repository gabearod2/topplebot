#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := esp-mpu9250

EXTRA_COMPONENT_DIRS := components/ahrs
						components/mpu9250
						components/micro_ros_esp_idf_component
						components/motor_control

include $(IDF_PATH)/make/project.mk

