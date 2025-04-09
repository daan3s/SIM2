#include <math.h>
#include <Servo.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X TOF = Adafruit_VL53L0X();    //declare time of flight

// remove buttons when bluetooth is ready
const int rightButton = 6;
const int upButton = 4;
const int downButton = 5;
const int leftButton = 3;
const int startButton = 7;

#define DIR_PIN  4  
#define STEP_PIN 2  
#define EN_PIN   8  

const int STEPS_PER_REV = 1600;  // 1/8 step mode
const float STEPS_PER_DEGREE = STEPS_PER_REV / 360.0;  // 4.44 steps per degree

const float stepperArmLengh = 192;  
const float servoArmLengh = 101;

int magnitude = map(51,0,100,stepperArmLengh-servoArmLengh,stepperArmLengh+servoArmLengh); //in millimeter | starts at 50% extended
const float magnitudePercent = (stepperArmLengh-servoArmLengh)/100;   //one percent between smallest and biggest magnitude
int magnitudeAngle = 90; // in deg


int magnitude = map(51,0,100,stepperArmLengh-servoArmLengh,stepperArmLengh+servoArmLengh); //in millimeter
const float magnitudePercent = (stepperArmLengh-servoArmLengh)/100;   //one percent between smallest and biggest magnitude
int magnitudeAngle = 90; // in deg

float stepperAngle = 0;   //upper arm's angle
float servoArmAngle = 0;   //lower arm's angle

Servo servoArm;
