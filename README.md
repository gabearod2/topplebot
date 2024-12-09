# ToppleBot

Software for ToppleBot microcontroller (ESP32), a scalable robot for autonomous balancing applications.

## Attributions

### Used Repositories

Forked from [here](https://github.com/miniben-90/mpu9250) for IMU integration. Additionally configured to include [micro-ROS for ESP](https://github.com/micro-ROS/micro_ros_espidf_component.git). 

### Calibration Resource

See more information about calibration [here](https://github.com/miniben-90/mpu9250).

## Start-Up

Clone this repository:

```
git clone --recurse-submodules https://github.com/gabearod2/topplebot.git &&
cd topplebot
```

Run the following command to follow your USB connections, before connecting the ESP32.

```
sudo dmesg --follow
```

Connect the ESP32 and record the name of the port. Assign privileges to the port with the ESP32 connected to it, using /dev/ttyUSB0 here for example:
```
sudo chmod 666 /dev/ttyUSB0
```

We will first have to calibrated the IMU. We will tell the ToppleBot to calibrate first in the configuration menu. In the menu, select "Component Config", then "MPU9250 Calibration," and set it to calibrate for first run. Building and writing the project to the ESP32, again using /dev/ttyUSB0 port as an example:

```
docker run -it --rm \
    --user espidf \
    --volume="/etc/timezone:/etc/timezone:ro" \
    -v "$(pwd)":/topplebot \
    -v /dev:/dev \
    --privileged \
    --workdir /topplebot \
    microros/esp-idf-microros:latest \
    /bin/bash -c "idf.py menuconfig build flash monitor"
```
(To exit the serial monitor, type `Ctrl-]`.)

If you do not know the IP adress of your control station, use the following command:

```
ifconfig
```

You will then have to set the corresponding calibration offsets to the main.c file. If you need to switch the GPIO pins of where you have connected the MPU9250, you can do this now as well. After that, you can again run the project, now configuring Micro-ROS in the menu. Once in the menu, select "Component Config", then "MPU9250 Calibration," and turn off the calibration. Then, select "micro-ROS settings", and enter the IP address of your control station, the Wifi SSID, and Wifi password. Quit the menu and let it build and write. Make sure you see that the ESP connects to the network. If not, check your SSID and password:

```
docker run -it --rm \
    --user espidf \
    --volume="/etc/timezone:/etc/timezone:ro" \
    -v "$(pwd)":/topplebot \
    -v /dev:/dev \
    --privileged \
    --workdir /topplebot \
    microros/esp-idf-microros:latest \
    /bin/bash -c "idf.py menuconfig build flash monitor"

```
If everything goes well, you will see the connection established and messages being sent. You can now disconnect the ESP32 and connect it to an external power source, micro-ROS has been configured on your ESP32!
