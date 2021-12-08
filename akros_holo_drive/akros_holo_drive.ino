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

#define MIN_PWM 50
#define MAX_PWM 255
#define MAX_RPM 150
#define MAX_RPM_GAIN 1.5
#define ENC_CPR 300
#define WHEEL_DIAMETER 0.075 //m
#define WHEELS_X_DISTANCE 0.0925 //m
#define WHEELS_Y_DISTANCE 0.225 //m

//motors 1:lf, 2:lb, 3:rb, 4:rf
Encoder enc1(19, 17);
Encoder enc2(18, 16);
Encoder enc3(20, 25);
Encoder enc4(21, 23);
int enPins[4] = {8, 9, 10, 11};
int inPins[8] = {32, 34, 36, 38, 42, 40, 46, 44};
double vels[4] = {0, 0, 0, 0};
double enc_vels[4] = {0, 0, 0, 0};
unsigned long prev_update_time[4];
long prev_encoder_ticks[4];

geometry_msgs::Twist twist_msg;
geometry_msgs::Point raw_vel_msg;

double read_rpm(int encoder_number, long encoder_ticks, int counts_per_rev){
  unsigned long current_time = millis();
  unsigned long dt = current_time - prev_update_time[encoder_number];

  double dt_min = (double)dt / 60000;
  double delta_ticks = encoder_ticks - prev_encoder_ticks[encoder_number];

  prev_update_time[encoder_number] = current_time;
  prev_encoder_ticks[encoder_number] = encoder_ticks;

  return (delta_ticks / counts_per_rev) / dt_min;
}

void holonomic_drive(double x, double y, double a){

  enc_vels[0] = read_rpm(0, enc1.read(), ENC_CPR);  //1:lf
  enc_vels[1] = read_rpm(1, enc2.read(), ENC_CPR);;  //2:lb
  enc_vels[2] = -read_rpm(2, enc3.read(), ENC_CPR);; //3:rb - reversed polarity
  enc_vels[3] = -read_rpm(3, enc4.read(), ENC_CPR);; //4:rf - reversed polarity

  float avg_rpm_x = ((enc_vels[0] + enc_vels[1] + enc_vels[2] + enc_vels[3])/4);
  raw_vel_msg.x = avg_rpm_x * PI * WHEEL_DIAMETER / 60; // m/s

  float avg_rpm_y = ((enc_vels[1] + enc_vels[3] - enc_vels[0] - enc_vels[2])/4);
  raw_vel_msg.y = avg_rpm_y * PI * WHEEL_DIAMETER / 60; // m/s

  float avg_rpm_a = ((enc_vels[2] + enc_vels[3] - enc_vels[0] - enc_vels[1])/4);
  raw_vel_msg.z = avg_rpm_a * PI * WHEEL_DIAMETER / ((WHEELS_X_DISTANCE/2 + WHEELS_Y_DISTANCE/2)*60); // rad/s

  float tangential = a * ((WHEELS_X_DISTANCE / 2) + (WHEELS_Y_DISTANCE / 2)); // m/s

  float x_rpm = constrain(x * 60 / (PI * WHEEL_DIAMETER), -MAX_RPM, MAX_RPM); // rotation per minute
  float y_rpm = constrain(y * 60 / (PI * WHEEL_DIAMETER), -MAX_RPM, MAX_RPM); // rotation per minute
  float a_rpm = constrain(tangential * 60 / (PI * WHEEL_DIAMETER), -MAX_RPM, MAX_RPM); // rotation per minute

  /*
  Serial.print("measured X: ");
  Serial.println(raw_vel_msg.x);
  Serial.print("measured Y: ");
  Serial.println(raw_vel_msg.y);
  Serial.print("measured A: ");
  Serial.println(raw_vel_msg.z * ((WHEELS_X_DISTANCE/2 + WHEELS_Y_DISTANCE/2)*60));
  Serial.print("Provided X: ");
  Serial.println(x_rpm);
  Serial.print("Provided Y: ");
  Serial.println(y_rpm);
  Serial.print("Provided A: ");
  Serial.println(a_rpm);
  */

  if(x!=0 || y!=0 || a!=0){
    vels[0] = x_rpm - y_rpm - a_rpm; //1:lf
    vels[1] = x_rpm + y_rpm - a_rpm; //2:lb
    vels[2] = x_rpm - y_rpm + a_rpm; //3:rb
    vels[3] = x_rpm + y_rpm + a_rpm; //4:rf

    //instead of mapping it directly, use PID to determine RPM->PWM conversion, 
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
        pwm = map(abs((int)vels[j]), 0, MAX_RPM_GAIN*MAX_RPM, MIN_PWM, MAX_PWM);
      }
      else{
        pwm = 0;
      }
      analogWrite(enPins[j], pwm);
    }
  }
  else{
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
ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", &twist_cb);
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

void twist_cb(const geometry_msgs::Twist& msg){
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
  if(nh.connected()){
    if(DEBUG){ colorWipe(setLEDColor(abs(twist_msg.linear.x*255), abs(twist_msg.linear.y*255), abs(twist_msg.angular.z*255))); }
    else{ colorWipe(setLEDColor(0, 127, 255)); }
  }
  FastLED.show();
  
  // holonomic_drive(0, 0, 0.5);
  holonomic_drive(twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z);
  delay(5);
  raw_vel_pub.publish(&raw_vel_msg);
  delay(5);
  nh.spinOnce();
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
