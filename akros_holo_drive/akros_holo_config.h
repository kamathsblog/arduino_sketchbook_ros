//DEBUG
#define DEBUG 0
#define OPEN_LOOP 0

//PID
#define KP 0.75
#define KI 0.50
#define KD 0.50
#define UPDATE_RATE 20 //ms

//NEOPIXEL
#define NEO_PIN        52
#define NEO_COUNT      6
#define NEO_BRIGHTNESS 50

//MODES
#define ESTOP 0
#define AUTO_T 1

//MOTOR CONTROL
#define MIN_PWM 60
#define MAX_PWM 245
#define MAX_RPM 150
#define OPEN_LOOP_GAIN 1.5
#define ENC_CPR 296
#define WHEEL_DIAMETER 0.0762 //m 3"
#define WHEELS_X_DISTANCE 0.093 //m
#define WHEELS_Y_DISTANCE 0.210 //m

#define SCALE_X 0.30
#define SCALE_Y 0.25
#define SCALE_RZ 1.57

//MOTOR CONTROL WIRING
//1:LF, 2:LB, 3:RB, 4:RF

//MOTOR/ENCODER 1: FRONT LEFT
#define M1_en 9
#define M1_a 37
#define M1_b 36
#define E1_a 19
#define E1_b 17

//MOTOR/ENCODER 2: BACK LEFT
#define M2_en 10
#define M2_a 39
#define M2_b 38
#define E2_a 18
#define E2_b 16

//MOTOR/ENCODER 3: BACK RIGHT
#define M3_en 11
#define M3_a 40
#define M3_b 41
#define E3_a 20
#define E3_b 24

//MOTOR/ENCODER 4: FRONT RIGHT
#define M4_en 12
#define M4_a 42
#define M4_b 43
#define E4_a 21
#define E4_b 22