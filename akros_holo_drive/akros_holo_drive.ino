#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <FastLED.h>
#include <Encoder.h>

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
#define ENC_CPR 300
#define WHEEL_DIAMETER 0.075 //m

//motors 1:lf, 2:lb, 3:rb, 4:rf
Encoder enc1(19, 17);
Encoder enc2(18, 16);
Encoder enc3(20, 25);
Encoder enc4(21, 23);
int enPins[4] = {8, 9, 10, 11};
int inPins[8] = {32, 34, 36, 38, 42, 40, 46, 44};
double vels[4] = {0, 0, 0, 0};

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
geometry_msgs::Twist twist_msg;
ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", &twist_cb);
geometry_msgs::Point raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

void twist_cb(const geometry_msgs::Twist& msg)
{
  twist_msg = msg;
}

void setup() {

  FastLED.addLeds<NEOPIXEL, NEO_PIN>(neopixel, NEO_COUNT);

  drive_estop();
  holonomic_drive(0, 0, 0);
  colorWipe(setLEDColor(0, 255, 0)); // green
  FastLED.show();

  nh.initNode();
  nh.subscribe(twist_sub);
  nh.advertise(raw_vel_pub);

}

void loop() {
  
  int vel1, vel2, vel3, vel4;
  vel1 = enc1.readRPM(ENC_CPR);
  vel2 = enc2.readRPM(ENC_CPR);
  vel3 = -1 * enc3.readRPM(ENC_CPR);
  vel4 = -1 * enc4.readRPM(ENC_CPR);

  float average_rps_x = ((vel1 + vel2 + vel3 + vel4)/4)/60.0;
  raw_vel_msg.x = average_rps_x * PI * WHEEL_DIAMETER; // m/s

  float average_rps_y = ((vel2 + vel4 - vel1 - vel3)/4)/60.0;
  raw_vel_msg.y = average_rps_y * PI * WHEEL_DIAMETER; // m/s

  float average_rps_a = ((vel3 + vel4 - vel1 - vel2)/4)/60.0;
  raw_vel_msg.z = average_rps_a * PI * WHEEL_DIAMETER; // m/s

  raw_vel_pub.publish(&raw_vel_msg);
  holonomic_drive(twist_msg.linear.x/MAX_X, twist_msg.linear.y/MAX_Y, twist_msg.angular.z/MAX_RZ);

  if(nh.connected()){
    if(DEBUG){ colorWipe(setLEDColor(abs(twist_msg.linear.x*255), abs(twist_msg.linear.y*255), abs(twist_msg.angular.z*255))); }
    else{ colorWipe(setLEDColor(0, 127, 255)); }
  }
  FastLED.show();
  
  nh.spinOnce();
  delay(20);
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
