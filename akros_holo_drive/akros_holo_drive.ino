#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define DEBUG 1

#define NEO_PIN        52
#define NEO_COUNT      6
#define NEO_BRIGHTNESS 30
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEO_COUNT, NEO_PIN, NEO_GRB + NEO_KHZ800);

#define V_MAX 255
#define L_GAIN 1
#define A_GAIN 0.5

#define SCALE_X 0.35
#define SCALE_Y 0.25
#define SCALE_RZ 1.85
#define SCALE 0.70

//motors 1:lf, 2:lb, 3:rb, 4:rf
int enPins[4] = {8, 9, 10, 11};
int inPins[8] = {32, 34, 36, 38, 42, 40, 46, 44};
double vels[4] = {0, 0, 0, 0};
geometry_msgs::Twist twist_msg;

double check_limits(double x){
  if(abs(x) < SCALE){
    if(x < 0){
      if(x > -1/2 * SCALE){ x = 0; }
      else{ x = -1 * SCALE; }
    }
    else{
      if(x != 0){
        if(x < 1/2 * SCALE){ x = 0; }
        else{ x = SCALE; }
      }
      else{
        x = 0;
      }
    }
  }
  return x;
}

void holonomic_drive(double x, double y, double z){
  
  double theta = atan2(y, x);
  double r     = sqrt((pow(x, 2) + pow(y, 2)));

  //x = check_limits(x);
  //y = check_limits(y);
  //z = check_limits(z);

  if(x!=0 || y!=0 || z!=0)
  {
    {
      vels[0] = V_MAX * (L_GAIN * r * cos(theta + PI/4) - A_GAIN * z) / (L_GAIN + A_GAIN); //1:lf
      vels[2] = V_MAX * (L_GAIN * r * cos(theta + PI/4) + A_GAIN * z) / (L_GAIN + A_GAIN); //3:rb
      vels[3] = V_MAX * (L_GAIN * r * sin(theta + PI/4) + A_GAIN * z) / (L_GAIN + A_GAIN); //4:rf
      vels[1] = V_MAX * (L_GAIN * r * sin(theta + PI/4) - A_GAIN * z) / (L_GAIN + A_GAIN); //2:lb
    }
  }
  else
  {
    drive_estop();
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
    analogWrite(enPins[j], abs((int)vels[j]));
  }
}

void drive_estop(){
  for(int i=0; i<4; i++){
      vels[i] = 0;
  }
  for(int k=0; k<8; k++){
    digitalWrite(inPins[k], LOW);
  }
}

ros::NodeHandle nh;

void twist_cb(const geometry_msgs::Twist& msg)
{
  twist_msg = msg;
}

ros::Subscriber<geometry_msgs::Twist> twist_sub("/cmd_vel", &twist_cb);

void setup() {

  strip.begin();
  strip.setBrightness(NEO_BRIGHTNESS);
  strip.show(); // Initialize all pixels to 'off'

  drive_estop();
  holonomic_drive(0, 0, 0);
  colorWipe(strip.Color(0, 255, 0), 10); // green

  nh.initNode();
  nh.subscribe(twist_sub);
}

void loop() {
  if(DEBUG){
    colorWipe(strip.Color(abs(twist_msg.linear.x*255), abs(twist_msg.linear.y*255), abs(twist_msg.angular.z*255)), 10);
  }
  else{
    if(nh.connected()){
      colorWipe(strip.Color(0, 127, 255), 10);
    }
  }
  
  holonomic_drive(twist_msg.linear.x/SCALE_X, twist_msg.linear.y/SCALE_Y, twist_msg.angular.z/SCALE_RZ);
  nh.spinOnce();
}

void colorWipe(uint32_t c, uint8_t wait) 
{
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}
