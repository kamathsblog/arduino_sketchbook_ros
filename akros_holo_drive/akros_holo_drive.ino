#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <Encoder.h>
#include <FastLED.h>
#include "akros_holo_config.h"

//Global variables and objects
geometry_msgs::Twist twist_msg;
geometry_msgs::Point raw_vel_msg;

double rpm_ref[4]    = {0, 0, 0, 0};
double rpm_meas[4]   = {0, 0, 0, 0};
double pwm_out[4]    = {0, 0, 0, 0};
double prev_error[4] = {0, 0, 0, 0};
double integral[4]   = {0, 0, 0, 0};
double derivative[4] = {0, 0, 0, 0};

int enPins[4] = {M1_en, M2_en, M3_en, M4_en};
int inPins[8] = {M1_a, M1_b, M2_a, M2_b, M3_a, M3_b, M4_a, M4_b};

Encoder enc1(E1_a, E1_b);
Encoder enc2(E2_a, E2_b);
Encoder enc3(E3_a, E3_b);
Encoder enc4(E4_a, E4_b);

CRGB neopixel[NEO_COUNT];

//Function prototypes
void pid(int motor, double reference, double measurement, double kp, double ki, double kd);
void spin_motor(int motor, double velocity);
void holonomic_drive(double x, double y, double a);
void drive_estop();
CRGB setLEDColor(uint8_t Rdata, uint8_t Gdata, uint8_t Bdata);
void colorWipe(CRGB in_led);

//Twist callback - stores latest received message in a global variable
void twist_cb(const geometry_msgs::Twist& msg){
  twist_msg = msg;
}

//ROS publishers and subscribers
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", &twist_cb);
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

void setup() {
  //Startup LEDs - green
  FastLED.addLeds<NEOPIXEL, NEO_PIN>(neopixel, NEO_COUNT);
  colorWipe(setLEDColor(0, 255, 0)); // green
  FastLED.show();
  
  //Startup Drive - set all pins and pwms to 0
  drive_estop();
  holonomic_drive(0, 0, 0);
  
  //Startup ROS - initialize node, subscribe and advertise to topics
  nh.initNode();
  nh.subscribe(twist_sub);
  nh.advertise(raw_vel_pub);

  //Connect to ROS
  while(!nh.connected()){ nh.spinOnce(); }
  
  //ROS connected
  colorWipe(setLEDColor(0, 127, 255)); //blue
  FastLED.show();
  delay(10);
}

void loop() {
  //Loop LEDs - set to blue if ROS is connected once.
  //If ROS gets disconnected, blue is retained
  if(nh.connected()){
    if(DEBUG){ colorWipe(setLEDColor(abs(twist_msg.linear.x*255), abs(twist_msg.linear.y*255), abs(twist_msg.angular.z*255))); }
    else{ colorWipe(setLEDColor(0, 127, 255)); } // blue
  }
  FastLED.show();
  
  //Loop Drive - drive based on global twist message values
  holonomic_drive(twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z);
      
  //Loop Publisher - publish computed velocities from holonomic_drive
  raw_vel_pub.publish(&raw_vel_msg);
      
  //Spin ROS node once, loop at 100Hz frequency
  nh.spinOnce();
  delay(20);
}

//PID Controller: Calculates output PWM for a motor using its reference and measured speeds
void pid(int motor, double reference, double measurement, double kp, double ki, double kd){
  double error;
  error = reference - measurement;
  integral[motor] += error;
  derivative[motor] += error - prev_error[motor];
  if(reference == 0 && error == 0){ integral[motor] = 0; } //reset integral
    
  double pid_out = kp*error + ki*integral[motor] + kd*derivative[motor];
  pwm_out[motor] = constrain(pid_out, -MAX_PWM, MAX_PWM);
  prev_error[motor] = error;
}

//Writes digital/PWM values to motor direction/enable pins for the specified motor
void spin_motor(int motor, double velocity){
  if(velocity > 0 ){
    digitalWrite(inPins[(motor+1)*2 - 2], HIGH);
    digitalWrite(inPins[(motor+1)*2 - 1], LOW);
  }
  else{
    digitalWrite(inPins[(motor+1)*2 - 2], LOW);
    digitalWrite(inPins[(motor+1)*2 - 1], HIGH);
  }  
  analogWrite(enPins[motor], map(abs((int)velocity), 0, MAX_PWM, MIN_PWM, MAX_PWM));   
}


//Using holonomic drive kinematics, drives individual motors based on input linear/angular velocities
void holonomic_drive(double x, double y, double a){
  rpm_meas[0] = enc1.readRPM(ENC_CPR);  //1:lf
  rpm_meas[1] = enc2.readRPM(ENC_CPR);  //2:lb
  rpm_meas[2] = -enc3.readRPM(ENC_CPR); //3:rb - reversed polarity
  rpm_meas[3] = -enc4.readRPM(ENC_CPR); //4:rf - reversed polarity

  float avg_rpm_x = ((rpm_meas[0] + rpm_meas[1] + rpm_meas[2] + rpm_meas[3])/4);
  float avg_rpm_y = ((rpm_meas[1] + rpm_meas[3] - rpm_meas[0] - rpm_meas[2])/4);
  float avg_rpm_a = ((rpm_meas[2] + rpm_meas[3] - rpm_meas[0] - rpm_meas[1])/4);
  
  raw_vel_msg.x = avg_rpm_x * PI * WHEEL_DIAMETER / 60; // m/s
  raw_vel_msg.y = avg_rpm_y * PI * WHEEL_DIAMETER / 60; // m/s
  raw_vel_msg.z = avg_rpm_a * PI * WHEEL_DIAMETER / ((WHEELS_X_DISTANCE/2 + WHEELS_Y_DISTANCE/2)*60); // rad/s

  float tangential = a * ((WHEELS_X_DISTANCE / 2) + (WHEELS_Y_DISTANCE / 2)); // m/s
  float x_rpm = constrain(x * 60 / (PI * WHEEL_DIAMETER), -MAX_RPM, MAX_RPM); // rotation per minute
  float y_rpm = constrain(y * 60 / (PI * WHEEL_DIAMETER), -MAX_RPM, MAX_RPM); // rotation per minute
  float a_rpm = constrain(tangential * 60 / (PI * WHEEL_DIAMETER), -MAX_RPM, MAX_RPM); // rotation per minute

  if(x!=0 || y!=0 || a!=0){
    rpm_ref[0] = x_rpm - y_rpm - a_rpm; //1:lf
    rpm_ref[1] = x_rpm + y_rpm - a_rpm; //2:lb
    rpm_ref[2] = x_rpm - y_rpm + a_rpm; //3:rb
    rpm_ref[3] = x_rpm + y_rpm + a_rpm; //4:rf
 
    for(int j=0; j<4; j++){
      pid(j, rpm_ref[j], rpm_meas[j], KP, KI, KD);
        
      double motor_speed = OPEN_LOOP ? OPEN_LOOP_GAIN*rpm_ref[j] : pwm_out[j];
      spin_motor(j, motor_speed);
    }
  }
  else{
    drive_estop();
  }
}


//Hard stops all motors - sets direction pins, reference velocities, pwm_values to zero
void drive_estop(){
  for(int i=0; i<4; i++){
      rpm_ref[i] = 0;
      digitalWrite(enPins[i], LOW);
  }
  for(int k=0; k<8; k++){
    digitalWrite(inPins[k], LOW);
  }
}


//Returns RGB values in CRGB format
CRGB setLEDColor(uint8_t Rdata, uint8_t Gdata, uint8_t Bdata){
    CRGB output(0, 0, 0);
    output.r = Rdata;
    output.g = Gdata;
    output.b = Bdata;
    return output;
}

//Sets all LEDs to CRGB color in sequence
void colorWipe(CRGB in_led){
  for(int i=0; i<NEO_COUNT; i++) {
    neopixel[i] = setLEDColor(in_led.r, in_led.g, in_led.b);
  }
}
