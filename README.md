# Arduino ROS Sketchbook

### Description
Arduino sketches for the AKROS Differential (Jetbot2) and Ackermann (Jetracer2) platforms. Both of the following use rosserial (assumed to be installed) to communicate with the ROS over serial.
* jetbot2_nano328o
  * Sketch for the Arduino Nano 328 (old bootloader) on the Jetbot2
  * Subscribes to the 'mode' topic of Int8 type
  * Subscribes to the 'loader' topic of Int8 type
  * Controls the servo motor based on the up/down command
  * Controls Neopixel strip LED colors according to the selected mode
* jetracer2_nano3280
  * Sketch for the Arduino Nano 328 (old bootloader) on the Jetbot2
  * Subscribes to the 'mode' topic of Int8 type and controls Neopixel strip LED colors (TODO)
  * Reads the hall effect sensor output via a hardware interrupt and computes speed which is published over rosserial
Code is uploaded using the Arduino IDE on the Jetson Nano with Neopixel and Rosserial libraries installed.

