# Arduino ROS Sketchbook

## Description
All of the following projects use either rosserial or micro-ROS (assumed to be installed/configured) to communicate with ROS and ROS2 respectively.

### Arduino sketches for the NVidia Jetbot/Jetracer platforms:
The Jetracer includes an NVidia Jetson Nano, and a Raspberry Pi camera on top of an off the shelf RC car. The RC car is modified to include a hall effect sensor based encoder. The electronics of this robot were then moved to a Jetbot chassis, but without the hall effect sensor encoder but with support for Neopixel LEDs. So, there is a lot of common code.

* jetbot2_nano328o
  * Sketch for the Arduino Nano 328 (old bootloader) on the Jetbot
  * Subscribes to the 'mode' topic of Int8 type
  * Controls Neopixel strip LED colors according to the selected mode
* jetracer2_nano3280
  * Sketch for the Arduino Nano 328 (old bootloader) on the Jetracer
  * Subscribes to the 'mode' topic of Int8 type and controls Neopixel strip LED colors (TODO)
  * Reads the hall effect sensor output via a hardware interrupt and computes speed which is published over rosserial

Both sketches are uploaded using the Arduino IDE from the Jetson Nano with Neopixel and Rosserial libraries installed. Tested using ROS Melodic.

### Arduino/Teensy sketches for the AKROS platform:
The AKROS robot is a four mecanum-wheeled robot with a LIDAR, an Intel Realsense T265 connected to a Raspberry Pi 4 (4GB). The RPi is connected to an Arduino Mega 2560 which is used to control the motor drivers to drive the robot. The Arduino Mega also reads from the four motor encoders and computes the linear and angular velocities of the robot. THe Arduino also connects to status LEDs. The code was uploaded using the Arduino IDE on the Jetson Nano with FastLED, [Encoder](https://github.com/adityakamath/Encoder) (modified) and ROSSerial libraries installed. The Arduino was replaced with a Teensy 4.1 board with Cytron motor drivers and PlatformIO was used to build and upload the code.

* akros_holo_drive
  * Sketch for the Arduino Mega 2560 on the AKROS robot
  * Subscribes to the 'cmd_vel/vector' topic of geometry_msgs::Point type
  * Subscribes to the 'cmd_vel/mode' topic of UInt32 type
  * Publishes to the 'raw_vec' topic of geometry:_msgs::Point type
  * Drives the 2xL298n based motor drivers based on the received vector
  * Reads the encoders and estimates the linear and angular velocity of robot
  * Implements closed loop feedback control - the motor speeds are controlled according to the measured velocities
  * Subscribes to mode messages and controls the status LEDs according to the selected mode
* akros_teensy_pio
  * Same functionality as akros_holo_drive
  * Uses a Teensy 4.1 breakout board
  * Drives 2x Cytron MDD3A motor drivers based on the closed loop system (slightly different operation compared to the L298n motor drivers)
  * Uses PlatformIO to build and upload the code.

Both sketches were uploaded from Windows 10 (using Arduino IDE and PlatformIO/VSCode) and tested using ROS Noetic.

### micro-ROS for Arduino sketches
Experiments with micro_ros_arduino using Arduino Portenta H7, Arduino Nano RP2040 Connect, and Teensy 4.1 with several sensors and actuators. The Arduino Portenta H7 is used with the Portenta Breakout board and the Teensy 4.1 is used with [this expansion board from Tindie](https://www.tindie.com/products/cburgess129/arduino-teensy41-teensy-41-expansion-board/)

* portenta_vl53l5cx_pcl_publisher
  * Sketch for the Arduino Portenta H7 (Lite Connected) and the Portenta Breakout with [this Sparkfun Qwiik ToF Imager](https://www.sparkfun.com/products/18642) which is based on the VL53L5CX 8x8 ToF sensor, with a maximum range of 4m.
  * Uses the Sparkfun VL53L5CX Library to read raw sensor data over I2C, and calculates the position of every detected point (x, y, z) w.r.t the sensor.
  * Uses the micro-ROS utilities to sync the time with the micro-ROS agent, and populates a PointCloud2 message, along with the calculated positions.
  * Publishes this PointCloud2 message using the UDP over Ethernet. (Currently, this example does not work with Serial or UDP over WiFi)
  * Also writes the raw measurements to the serial port as comma separated values, which is then used by an accompanying Processing app to visualize.

This sketch was uploaded from Windows 10 (using the Arduino IDE with micro_ros_arduino v2.0.5-galactic) and tested using ROS2 Galactic.