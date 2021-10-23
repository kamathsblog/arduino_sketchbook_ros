#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <FastLED.h>

#define DEBUG 0

#define NEO_PIN        52
#define NEO_COUNT      6
#define NEO_BRIGHTNESS 50

CRGB neopixel[NEO_COUNT];

#define V_MAX 255
#define L_GAIN 1
#define A_GAIN 0.5

#define MAX_X 0.35
#define MAX_Y 0.25
#define MAX_RZ 1.85
#define MIN_PWM 50

//motors 1:lf, 2:lb, 3:rb, 4:rf
int enPins[4] = {8, 9, 10, 11};
int inPins[8] = {32, 34, 36, 38, 42, 40, 46, 44};
double vels[4] = {0, 0, 0, 0};
geometry_msgs::Twist twist_msg;

void holonomic_drive(double x, double y, double z){
  
  double theta = atan2(y, x);
  double r     = sqrt((pow(x, 2) + pow(y, 2)));

  if(x!=0 || y!=0 || z!=0)
  {
    {
      vels[0] = V_MAX * (L_GAIN * r * cos(theta + PI/4) - A_GAIN * z) / (L_GAIN + A_GAIN); //1:lf
      vels[2] = V_MAX * (L_GAIN * r * cos(theta + PI/4) + A_GAIN * z) / (L_GAIN + A_GAIN); //3:rb
      vels[3] = V_MAX * (L_GAIN * r * sin(theta + PI/4) + A_GAIN * z) / (L_GAIN + A_GAIN); //4:rf
      vels[1] = V_MAX * (L_GAIN * r * sin(theta + PI/4) - A_GAIN * z) / (L_GAIN + A_GAIN); //2:lb
    }
    for(int j=0; j<4; j++){
      if(vels[j] > 0 ){
        digitalWrite(inPins[(j+1)*2 - 2], HIGH);
        digitalWrite(inPins[(j+1)*2 - 1], LOW);
      }
      else{
        digitalWrite(inPins[(j+1)*2 - 2], LOW);
        digitalWrite(inPins[(j+1)*2 - 1], HIGH);
      }
      int pwm;
      if(abs((int)vels[j])!=0){
        pwm = map(abs((int)vels[j]), 0, 255, MIN_PWM, 255);
      }
      else{
        pwm = 0;
      }
      analogWrite(enPins[j], pwm);
    }
  }
  else
  {
    drive_estop();
  }
}

void drive_estop(){
  for(int i=0; i<4; i++){
      vels[i] = 0;
  }
  for(int k=0; k<8; k++){
    digitalWrite(inPins[k], LOW);
    digitalWrite(enPins[k], LOW);
  }
}

ros::NodeHandle nh;

void twist_cb(const geometry_msgs::Twist& msg)
{
  twist_msg = msg;
}

ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", &twist_cb);

void setup() {

  FastLED.addLeds<NEOPIXEL, NEO_PIN>(neopixel, NEO_COUNT);

  drive_estop();
  holonomic_drive(0, 0, 0);
  colorWipe(setLEDColor(0, 255, 0)); // green
  FastLED.show();

  nh.initNode();
  nh.subscribe(twist_sub);
}

void loop() {
  
  if(nh.connected()){
    if(DEBUG){
      colorWipe(setLEDColor(abs(twist_msg.linear.x*255), abs(twist_msg.linear.y*255), abs(twist_msg.angular.z*255)));
    }
    else{
      colorWipe(setLEDColor(0, 127, 255));
    }
  }
  else{
    nh.spinOnce();
  }

  FastLED.show();
  nh.spinOnce();
  holonomic_drive(twist_msg.linear.x/MAX_X, twist_msg.linear.y/MAX_Y, twist_msg.angular.z/MAX_RZ);
  delay(10);
}

CRGB setLEDColor(uint8_t Rdata, uint8_t Gdata, uint8_t Bdata){
    CRGB output(0, 0, 0);
    output.r = Rdata;
    output.g = Gdata;
    output.b = Bdata;
    return output;
}

void colorWipe(CRGB in_led){
  for(int i=0; i<NEO_COUNT; i++) {
    neopixel[i] = setLEDColor(in_led.r, in_led.g, in_led.b);
  }
}
