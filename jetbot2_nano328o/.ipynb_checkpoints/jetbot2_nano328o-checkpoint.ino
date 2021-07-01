/*
==========
Arduino Nano code for the Jetbot2
This code implements the following functionality:
  > NeoPixel LED control to indicate ROSCar modes (subscriber)
  > ROS Serial communication with Host PC (Jetson Nano)
by Aditya Kamath
adityakamath.github.io
github.com/adityakamath
==========
*/

#include <ros.h>
#include <std_msgs/Int8.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN    6
#define LED_COUNT  8
#define LED_BRIGHTNESS 50
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

ros::NodeHandle nh;
std_msgs::Int8 mode_msg;

void message_callback(const std_msgs::Int8& msg){
    switch(msg.data){
        case 0: 
            colorWipe(strip.Color(255, 255, 0), 10); //yellow
            break;
        case 1: 
            colorWipe(strip.Color(255, 0, 0), 10); //red
            break;
        case 2: 
            colorWipe(strip.Color(0, 255, 0), 10); //green
            break;
        case 3: 
            colorWipe(strip.Color(0, 0, 255), 10); //blue
            break;
        default:
            colorWipe(strip.Color(0, 0, 0), 10); //off
            break;     
    }
}

ros::Subscriber<std_msgs::Int8> mode_sub("/joy_node/mode", &message_callback);

void setup(){

   //setup neopixel strip
   strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
   strip.show();            // Turn OFF all pixels ASAP
   strip.setBrightness(55); // Set BRIGHTNESS to about 1/5 (max = 255)

   //car bootup sequence
   mode_msg.data = 0;
   loader_msg.data = 0;
   colorWipe(strip.Color(127, 0, 255), 10); //purple
   
   nh.initNode();
   nh.subscribe(mode_sub);
}

void loop(){
  if(!nh.connected()){
    colorWipe(strip.Color(127, 0, 255), 10); //purple
  }
  nh.spinOnce();
  delay(10);
}

//wipe user specified color over all 8 LEDS
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}
