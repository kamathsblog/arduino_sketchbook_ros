/*
==========
Arduino Nano code for the ROSCar (Jetracer2)
This code implements the following functionality:
  > Odometry (linear speed/distance) using a hall effect sensor
  > NeoPixel LED control to indicate ROSCar modes
  > ROS Serial communication with Host PC (Jetson Nano)
by Aditya Kamath
adityakamath.github.io
github.com/adityakamath
==========
*/

#include <ros.h>
#include <std_msgs/Float64.h>
#include <Adafruit_NeoPixel.h>

ros::NodeHandle nh;
std_msgs::Float64 odom_msg;
ros::Publisher odom_pub("/odom_node/odom_linear", &odom_msg);

#define LED_PIN    6
#define LED_COUNT  8
#define LED_BRIGHTNESS 50
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

#define TICKS_PER_ROTATION 12 //6 magnets on the shaft ring * 2 (two rising edges per magnet)
#define TICKS_PER_10CM     18 //calibrated by testing --> define this in config file during calibration, send to arduino via serial in setup sequence

volatile byte ticks;
unsigned long time_old;
double time_elapsed, shaft_rpm, lin_speed;

void setup(){
   //Serial.begin(115200);
   attachInterrupt(0, detect_magnet, RISING);//Initialize the interrupt pin (Arduino digital pin 2)
   
   ticks = 0;
   time_old = 0;
   time_elapsed = 0;
   shaft_rpm = 0;
   lin_speed = 0;

   //setup neopixel strip
   strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
   strip.show();            // Turn OFF all pixels ASAP
   strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

   //car bootup sequence
   colorWipe(strip.Color(255, 0, 0), 50); //red
   delay(50*52); //enough time for the host PC to boot up
   colorWipe(strip.Color(0, 255, 0), 50); //green
   delay(50*52); //enough time for the host PC to launch services
   colorWipe(strip.Color(0, 0, 255), 50); //blue
   
   nh.initNode();
   nh.advertise(odom_pub);
}

void loop(){
  //if(ticks >= TICKS_PER_ROTATION){ //update when 1 shaft rotation occurs
      
      //calculate time since last shaft rotation
      time_elapsed = (millis() - time_old)/1000.00; //seconds
      
      //calculate RPM using elapsed time and number of ticks since last shaft rotation
      shaft_rpm = (ticks*60.00) / (TICKS_PER_ROTATION*time_elapsed); //rotations per minute
      
      //calculate linear speed using the TICKS_PER_10CM ratio
      lin_speed = ticks / (TICKS_PER_10CM*time_elapsed*10.00); //meters per second

      //store current time and reset ticks to 0
      time_old = millis();
      ticks = 0;

      //publish via ROS serial
      odom_msg.data = lin_speed;
      odom_pub.publish( &odom_msg );
      nh.spinOnce();
      
      delay(1000); //update every 1 second
  //}
}
 
//ISR to increment tick value every time a rising edge is sensed by the interrupt pin
void detect_magnet(){
   ticks++;
}

//wipe user specified color over all 8 LEDS
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}
