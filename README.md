# Arduino ROS Sketchbook

## Description
All of the following projects use rosserial (assumed to be installed) to communicate with ROS over serial. Tested with ROS1 Noetic running on a Jetson Nano/Raspberry Pi 4, with an Arduino Nano 328/Arduino Mega 2560/Teensy 4.1.

### Arduino sketches for the NVidia Jetbot/Jetracer platforms:  
The Jetracer includes an NVidia Jetson Nano, and a Raspberry Pi camera on top of an off the shelf RC car. The RC car is modified to include a hall effect sensor based encoder. The electronics of this robot were then moved to a Jetbot chassis, but without the hall effect sensor encoder but with support for Neopixel LEDs. So, there is a lot of common code

* jetbot2_nano328o
  * Sketch for the Arduino Nano 328 (old bootloader) on the Jetbot
  * Subscribes to the 'mode' topic of Int8 type
  * Controls Neopixel strip LED colors according to the selected mode
* jetracer2_nano3280
  * Sketch for the Arduino Nano 328 (old bootloader) on the Jetracer
  * Subscribes to the 'mode' topic of Int8 type and controls Neopixel strip LED colors (TODO)
  * Reads the hall effect sensor output via a hardware interrupt and computes speed which is published over rosserial
Code is uploaded using the Arduino IDE on the Jetson Nano with Neopixel and Rosserial libraries installed.

### Arduino/Teensy sketches for the AKROS platform:
The AKROS robot is a four mecanum-wheeled robot with a LIDAR, an Intel Realsense T265 connected to a Raspberry Pi 4 (4GB). The RPi is connected to an Arduino Mega 2560 which is used to control the motor drivers to drive the robot. The Arduino Mega also reads from the four motor encoders and computes the linear and angular velocities of the robot. THe Arduino also connects to status LEDs. The code was uploaded using the Arduino IDE on the Jetson Nano with FastLED, [Encoder](https://github.com/adityakamath/Encoder) (modified) and Rosserial libraries installed. The Arduino was replaced with a Teensy 4.1 board with Cytron motor drivers and PlatformIO was used to build and upload the code.

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