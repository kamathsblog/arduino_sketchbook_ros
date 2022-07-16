/*
==========
Teensy 4.1 code for the AKROS robot with PlatformIO
This code implements the following functionality:
  > NeoPixel LED control to indicate operating modes (subscriber)
  > Drives the mecanum wheel robot with Cytron MDD3A motor drivers based on Twist values and the operating mode (subscriber)
  > Measures wheel encoders and calculates the linear and angular velocities of the robot (publisher)
  > ROSSerial communication with Host PC (Raspberry Pi 4 with ROS Noetic)
Author: Aditya Kamath
adityakamath.github.io
github.com/adityakamath
==========
*/

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Point.h>
#include <Encoder.h>
#include <FastLED.h>
#include "akros_holo_teensy_config.h"

//Global variables and objects
geometry_msgs::Point twist_msg;
geometry_msgs::Point raw_vel_msg;
bool mode_msg[5] = {false, false, false, false, false};

double rpm_ref[4]    = {0, 0, 0, 0};
double rpm_meas[4]   = {0, 0, 0, 0};
double pwm_out[4]    = {0, 0, 0, 0};
double prev_error[4] = {0, 0, 0, 0};
double integral[4]   = {0, 0, 0, 0};
double derivative[4] = {0, 0, 0, 0};

int inPins[8] = {M1_a, M1_b, M2_a, M2_b, M3_a, M3_b, M4_a, M4_b};

Encoder enc1(E1_a, E1_b);
Encoder enc2(E2_a, E2_b);
Encoder enc3(E3_a, E3_b);
Encoder enc4(E4_a, E4_b);

CRGB neopixel[NEO_COUNT];

//Function prototypes
void bootupLEDsequence(int max_steps, int step);
void pid(int motor, double reference, double measurement, double kp, double ki, double kd);
void spinMotor(int motor, double velocity);
void holoDrive(double x, double y, double a);
void eStop();
CRGB toCRGB(uint8_t Rdata, uint8_t Gdata, uint8_t Bdata);
void setNeopixel(CRGB in_led);

//Twist callback - stores latest received message in a global variable
void twist_cb(const geometry_msgs::Point& msg){
  twist_msg = msg;
}

//Mode callback - constructs mode message and stores in a global variable
void mode_cb(const std_msgs::UInt32& msg){
  std_msgs::UInt32 message = msg;
  for(int i=0; i<5; i++){
    mode_msg[i] = message.data%10 != 0 ? true:false;
    message.data /= 10;
  }
}

//ROS publishers and subscribers
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Point> twist_sub("cmd_vel/vector", &twist_cb);
ros::Subscriber<std_msgs::UInt32> mode_sub("cmd_vel/mode", &mode_cb);
ros::Publisher raw_vel_pub("raw_vec", &raw_vel_msg);

void setup()
{
  //Startup LEDs - green
  FastLED.addLeds<NEOPIXEL, NEO_PIN>(neopixel, NEO_COUNT);
  bootupLEDsequence(5, 1);

  //Startup Drive - set all pins to 0
  eStop();
  bootupLEDsequence(5, 2);

  //Startup ROS - initialize node, subscribe and advertise to topics
  nh.initNode();
  nh.subscribe(twist_sub);
  nh.subscribe(mode_sub);
  nh.advertise(raw_vel_pub);
  bootupLEDsequence(5, 3);

  //Connect to ROS
  while(!nh.connected())
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    bootupLEDsequence(5, 4);
    nh.spinOnce();
  }

  //ROS connected
  bootupLEDsequence(5, 5);
}

void loop()
{

  if(mode_msg[ESTOP] == true){ // STOP! - red
    eStop();
    setNeopixel(toCRGB(255, 0, 0));

    holoDrive(0, 0, 0);
    nh.spinOnce();
  }
  else{
    if(mode_msg[AUTO_T] == true){ // AUTO
      setNeopixel(toCRGB(0, 75, 255));
    }
    else{ // TELEOP
      if(DEBUG){
        (toCRGB(abs(twist_msg.x*MAX_PWM/SCALE_X), abs(twist_msg.y*MAX_PWM/SCALE_Y), abs(twist_msg.z*MAX_PWM/SCALE_RZ)));
      }
      else{
        setNeopixel(toCRGB(0, 255, 75));
      }
    }
    //Loop Drive - drive based on global twist message values
    holoDrive(twist_msg.x, twist_msg.y, twist_msg.z);
    nh.spinOnce();
  }

  //Loop Publisher - compute velocities from holoDrive and publish
  float avg_rpm_x = ((rpm_meas[0] + rpm_meas[1] + rpm_meas[2] + rpm_meas[3])/4);
  float avg_rpm_y = ((rpm_meas[1] + rpm_meas[3] - rpm_meas[0] - rpm_meas[2])/4);
  float avg_rpm_a = ((rpm_meas[2] + rpm_meas[3] - rpm_meas[0] - rpm_meas[1])/4);

  raw_vel_msg.x = avg_rpm_x * PI * WHEEL_DIAMETER / 60; // m/s
  raw_vel_msg.y = avg_rpm_y * PI * WHEEL_DIAMETER / 60; // m/s
  raw_vel_msg.z = avg_rpm_a * PI * WHEEL_DIAMETER / ((WHEELS_X_DISTANCE/2 + WHEELS_Y_DISTANCE/2)*60); // rad/s

  raw_vel_pub.publish(&raw_vel_msg);

  FastLED.show();
  nh.spinOnce();

  //loop at 50Hz frequency
  delay(20);
}

//Sets LED color wipe according to number of boot steps, and the current step provided by user
void bootupLEDsequence(int max_steps, int step)
{
  setNeopixel(toCRGB(0, MAX_PWM*step/max_steps, MAX_PWM*step/(2.5*max_steps)));
  FastLED.show();
  delay(100);
}

//PID Controller: Calculates output PWM for a motor using its reference and measured speeds
void pid(int motor, double reference, double measurement, double kp, double ki, double kd)
{
  double error = reference - measurement;
  integral[motor] += error;
  derivative[motor] += error - prev_error[motor];
  if(reference == 0 && error == 0){ integral[motor] = 0; } //reset integral

  double pid_out = kp*error + ki*integral[motor] + kd*derivative[motor];
  pwm_out[motor] = constrain(pid_out, -MAX_PWM, MAX_PWM);
  prev_error[motor] = error;
}

//Writes PWM values to motor pins for the specified motor
void spinMotor(int motor, int pwm, bool direction)
{
  if(direction == FORWARD)
  {
    analogWrite(inPins[2*motor], pwm);
    analogWrite(inPins[2*motor + 1], 0);
  }
  else if(direction == BACKWARD)
  {
    analogWrite(inPins[2*motor], 0);
    analogWrite(inPins[2*motor + 1], pwm);
  }
}

//Drives motor: If PWM is above a particular threshold, spins motor at half speed for specified time, then at full speed.
void drive_motor_ramp(int motor, double velocity, int threshold)
{
  bool direction = FORWARD;
  if(velocity < 0)
  {
    direction = BACKWARD;
  }
  int motor_pwm = constrain(abs((int)velocity), MIN_PWM, MAX_PWM);

  if(motor_pwm > threshold*MAX_PWM/100)
  {
    spinMotor(motor, motor_pwm/2, direction);
    delay(RAMP_DELAY);
  }
  spinMotor(motor, motor_pwm, direction);
}

//Using holonomic drive kinematics, drives individual motors based on input linear/angular velocities
void holoDrive(double x, double y, double a)
{
  float tangential = a * ((WHEELS_X_DISTANCE / 2) + (WHEELS_Y_DISTANCE / 2)); // m/s
  float x_rpm = constrain(x * 60 / (PI * WHEEL_DIAMETER), -MAX_RPM, MAX_RPM); // rotation per minute
  float y_rpm = constrain(y * 60 / (PI * WHEEL_DIAMETER), -MAX_RPM, MAX_RPM); // rotation per minute
  float a_rpm = constrain(tangential * 60 / (PI * WHEEL_DIAMETER), -MAX_RPM, MAX_RPM); // rotation per minute

  if(x!=0 || y!=0 || a!=0)
  {
    rpm_ref[0] = x_rpm - y_rpm - a_rpm; //1:lf
    rpm_ref[1] = x_rpm + y_rpm - a_rpm; //2:lb
    rpm_ref[2] = x_rpm - y_rpm + a_rpm; //3:rb
    rpm_ref[3] = x_rpm + y_rpm + a_rpm; //4:rf

    for(int j=0; j<4; j++)
    {
      pid(j, rpm_ref[j], rpm_meas[j], KP, KI, KD);
      double motor_speed = OPEN_LOOP ? OPEN_LOOP_GAIN*rpm_ref[j] : pwm_out[j];
      drive_motor_ramp(j, motor_speed, RAMP_THRESHOLD);
    }
  }
  else
  {
    eStop();
  }

  rpm_meas[0] = enc1.readRPM(ENC_CPR);  //1:lf
  rpm_meas[1] = enc2.readRPM(ENC_CPR);  //2:lb
  rpm_meas[2] = -enc3.readRPM(ENC_CPR); //3:rb - reversed polarity
  rpm_meas[3] = -enc4.readRPM(ENC_CPR); //4:rf - reversed polarity
}

//Hard stops all motors - reference velocities, motor pins to zero
void eStop()
{
  for(int i=0; i<4; i++)
  {
      rpm_ref[i] = 0;
      analogWrite(inPins[2*i], 0);
      analogWrite(inPins[2*i+1], 0);
  }
}

//Returns RGB values in CRGB format
CRGB toCRGB(uint8_t Rdata, uint8_t Gdata, uint8_t Bdata)
{
    CRGB output(0, 0, 0);
    output.r = Rdata;
    output.g = Gdata;
    output.b = Bdata;
    return output;
}

//Sets all LEDs to CRGB color in sequence
void setNeopixel(CRGB in_led)
{
  for(int i=0; i<NEO_COUNT; i++)
  {
    neopixel[i] = toCRGB(in_led.r, in_led.g, in_led.b);
  }
}