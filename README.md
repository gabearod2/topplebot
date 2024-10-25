# ToppleBot

Software for ToppleBot, a scalable robot for space applications.

## Attributions

Forked from [here](https://github.com/miniben-90/mpu9250) for IMU integration. Additionally configured to include [micro-ROS for ESP](https://github.com/micro-ROS/micro_ros_espidf_component.git). 

## Calibration 

See more information about calibration [here](https://github.com/miniben-90/mpu9250).

## Start-Up

Build the project and flash it to the board, then run monitor tool to view serial output:

```
source ~/esp/v5.2.3/esp-idf/export.sh
idf.py menuconfig
```

Select "Component Config", then "MPU9250 Calibration," and set it to calibrate for first run. If you do not have correct permissions, or do not know the name of the port:

```
sudo dmesg --follow
ls -l /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB0
```

Then, build the project, and flash it onto your ESP32.

```
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

(To exit the serial monitor, type `Ctrl-]`.)

Then, to run the IMU after copying results in main.c, do the following. Select "Component Config", then "MPU9250 Calibration," and set it to NOT calibrate. Similarly as before:

```
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

You should now see the IMU data.
