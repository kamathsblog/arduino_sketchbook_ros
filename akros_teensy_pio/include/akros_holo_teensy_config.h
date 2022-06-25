//DEBUG
#define DEBUG 0
#define OPEN_LOOP 0

//PID
#define KP 0.75
#define KI 0.50
#define KD 0.50
#define UPDATE_RATE 20 //ms

//NEOPIXEL
#define NEO_PIN        24 //52
#define NEO_COUNT      6
#define NEO_BRIGHTNESS 50

//MODES
#define ESTOP 0
#define AUTO_T 1
#define PLAY_WP 2
#define PLAY_T 3
#define RECORD 4

//MOTOR CONTROL
#define MIN_PWM 50
#define MAX_PWM 200
#define MAX_RPM 125
#define OPEN_LOOP_GAIN 1.5
#define ENC_CPR 296
#define WHEEL_DIAMETER 0.0762 //m 3"
#define WHEELS_X_DISTANCE 0.093 //m
#define WHEELS_Y_DISTANCE 0.210 //m

#define SCALE_X 0.25
#define SCALE_Y 0.175
#define SCALE_RZ 1

#define FORWARD true
#define BACKWARD false
#define RAMP_THRESHOLD 75 //percentage of MAX_PWM
#define RAMP_DELAY 20 //delay in ms

//MOTOR CONTROL WIRING
//1:LF, 2:LB, 3:RB, 4:RF

//MOTOR/ENCODER 1: FRONT LEFT
#define M1_a 2  //37
#define M1_b 3  //36
#define E1_a 31 //19
#define E1_b 29 //17

//MOTOR/ENCODER 2: BACK LEFT
#define M2_a 36 //39
#define M2_b 37 //38
#define E2_a 30 //18
#define E2_b 28 //16

//MOTOR/ENCODER 3: BACK RIGHT
#define M3_a 4  //40
#define M3_b 5  //41
#define E3_a 32 //20
#define E3_b 9  //24

//MOTOR/ENCODER 4: FRONT RIGHT
#define M4_a 6  //42
#define M4_b 7  //43
#define E4_a 33 //21
#define E4_b 8  //22