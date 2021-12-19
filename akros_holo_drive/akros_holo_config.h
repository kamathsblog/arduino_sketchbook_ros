//DEBUG
#define DEBUG 0
#define OPEN_LOOP 0

//NEOPIXEL
#define NEO_PIN        52
#define NEO_COUNT      6
#define NEO_BRIGHTNESS 50

//MOTOR CONTROL
#define MIN_PWM 50
#define MAX_PWM 255
#define MAX_RPM 150
#define OPEN_LOOP_GAIN 1.5
#define ENC_CPR 300
#define WHEEL_DIAMETER 0.075 //m
#define WHEELS_X_DISTANCE 0.0925 //m
#define WHEELS_Y_DISTANCE 0.225 //m
#define KP 0.65
#define KI 0.25 
#define KD 0.50

//MOTOR CONTROL WIRING
//1:LF, 2:LB, 3:RB, 4:RF

//MOTOR/ENCODER 1: FRONT LEFT
#define M1_en 8
#define M1_a 32 
#define M1_b 34
#define E1_a 19
#define E1_b 17

//MOTOR/ENCODER 2: BACK LEFT
#define M2_en 9
#define M2_a 36
#define M2_b 38
#define E2_a 18
#define E2_b 16

//MOTOR/ENCODER 3: BACK RIGHT
#define M3_en 10
#define M3_a 42
#define M3_b 40
#define E3_a 20
#define E3_b 25

//MOTOR/ENCODER 4: FRONT RIGHT
#define M4_en 11
#define M4_a 46
#define M4_b 44
#define E4_a 21
#define E4_b 23
