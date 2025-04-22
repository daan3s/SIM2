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
#define FSR_PIN A0

int currentStepperAngle; 
const int STEPS_PER_REV = 1600;  // 1/8 step mode
const float STEPS_PER_DEGREE = STEPS_PER_REV / 360.0;  // 4.44 steps per degree

const int thresholdPress = 512;

const float stepperArmLengh = 192;  
const float servoArmLengh = 101;

int magnitude = map(51,0,100,stepperArmLengh-servoArmLengh,stepperArmLengh+servoArmLengh); //in millimeter | starts at 50% extended
const float magnitudePercent = (stepperArmLengh-servoArmLengh)/100;   //one percent between smallest and biggest magnitude
int magnitudeAngle = 90; // in deg

float stepperAngle = 0;   //upper arm's angle
float servoArmAngle = 0;   //lower arm's angle

bool debugMode;

Servo servoArm;
Servo servoZ;
Servo servoGrip;

void setup() {
  // put your setup code here, to run once:
  
  pinMode(FSR_PIN, INPUT);
  pinMode(DIR_PIN, OUTPUT);  //stepper pinout
  pinMode(STEP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);  // Enable motor driver

  servoArm.attach(10); //lower arm
  servoZ.attach(13); //temp pin for z servo
  servoGrip.attach(9);

  Serial.begin(9600);
  Serial.println("");
  if (!TOF.begin()) {   //initialise serial for ToF
    Serial.println(F("Failed to boot Time of Flight sensor"));
    while(1);
  }

  Serial.println("start in debug mode?   (type 1 for yes)");
  switch(DataIN()) {
    case 1:
      debugMode = 1;
      Serial.println("debug mode !");
      break;
    default:
      debugMode = 0;
      Serial.println("normal mode !");
  }
}


void loop() {
  if (debugMode){
    //debugmode 
    //manualy execute functions
      Serial.println("list of functions :  1,inverseK   2,gototarget   3,readTOF  4,grab  5,sweep  6,steppertoangle  7,servoArm  8,Zservo  9,gripServo");
      Serial.println("please input the number next to the function to select it");
  switch(DataIN()) {
    case 1:
      //inverseK
      Serial.println("please input [target]magnitudeAngle (0-360) : ");
      magnitudeAngle = DataIN();
      Serial.println("please input [target]magnitude (0-100) : ");
      magnitude = DataIN()*magnitudePercent;
      inverseK(magnitudeAngle,magnitude);
      break;
    case 2:
      //gototarget
      Serial.println("please input [target]magnitude (0-100) : ");
      goToTarget(DataIN());
      
    break;

    case 3:
      //readTOF
      Serial.println("please input number of iterations : ");
      Serial.println(readTOF(DataIN()));
    break;

    case 4:
      //grab
      Serial.println("grabing...");
      grab();
    break;

    case 5:
      //sweep
      Serial.println("incomplete");
    break;

    case 6:
      //steppertoangle
      Serial.println("please input angle for stepper motor : ");
      stepperAngle = DataIN();
      stepperToAngle(stepperAngle);
      
    break;

    case 7:
      //servoArm
      Serial.println("please input angle for servo : ");
      servoArmAngle = DataIN();
      servoArm.write(servoArmAngle);    

    break;

    case 8:
      //Zservo
      Serial.println("please input angle for servo : ");
      servoZ.write(DataIN());
      

    break;

    case 9:
      //gripServo
      Serial.println("please input angle for servo : ");
      servoGrip.write(DataIN());

    break;

    default:
      Serial.println("command not recognised");
    }
  }else{
    //normal mode
    //automicly does everything
    // INCOMPLETE !!! 
    

    if (digitalRead(startButton) == LOW){
      delay(10);
      goToTarget(readTOF(10));
    }

      inverseK(magnitudeAngle,magnitude);

      stepperToAngle(stepperAngle+magnitudeAngle);   //will be replaced with the stepper motor
      servoArm.write(servoArmAngle);    //i put my servo backards

  }
}



int DataIN(){
  //need to make this not part advance until a command has been given
  int tenp;
  Serial.print("input a number pls : ");
  
  while(Serial.available() == 0){} //stops everything until there's input at serial 
    
  String input = Serial.readStringUntil('\n');  
  input.trim();  

  if(isNumber(input)){
    tenp = input.toInt();
    Serial.println(tenp);
    return(tenp);
  }
    
  
}


bool isNumber(String str) {
    if (str.length() == 0) return false;
    for (unsigned int i = 0; i < str.length(); i++) {
        if (!isDigit(str[i])) {
            return false;
        }
    }
    return true;
}

void stepperToAngle(int targetAngle) {

    float angleDifference = targetAngle - currentStepperAngle;
    int stepsToMove = abs(angleDifference) * STEPS_PER_DEGREE;  

    Serial.print("Moving ");
    Serial.print(stepsToMove);
    Serial.println(" steps");

    digitalWrite(DIR_PIN, angleDifference > 0 ? HIGH : LOW);

    int minDelay = 5000; // higher valeue » Slow movement for high precision
    int maxDelay = 5500;  // if maxdelay is higher than mindelay gradual acceleration becomes gradual decelaretion  

    for (int i = 0; i < stepsToMove; i++) {
        int stepDelay = map(i, 0, stepsToMove, minDelay, maxDelay); // Gradual acceleration
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(stepDelay);
    }

    currentStepperAngle = targetAngle; 
    Serial.println("Movement complete!");
}

void inverseK(float ang,float mag)
{
  stepperAngle = (acos((pow(stepperArmLengh,2) + pow(mag,2) - pow(servoArmLengh,2)) / (2.0*stepperArmLengh*mag))) * (180.0 / PI); //upper arm (sloser to the base) angle to magnitude line
  servoArmAngle = (acos(((stepperArmLengh*stepperArmLengh) + (servoArmLengh*servoArmLengh) - (mag*mag)) / (2.0*stepperArmLengh*servoArmLengh))) * (180.0 / PI); //lower arm angle to uper arm
    
}

void goToTarget(float mesuredDistance){ 

  mesuredDistance = mesuredDistance*1.08; //slightly incresed it to give it a satisfying smack|||||||||||remove later|||||||||||||||||||
  float newMagnitude;
  float newAngle;
  float sensAngle = abs(stepperAngle + servoArmAngle -90); //getting the angle between the magnitude line and the sensor direction 

  newMagnitude = sqrt(pow(magnitude,2)+pow(mesuredDistance,2)-2*mesuredDistance*magnitude*cos((sensAngle/180)* PI));
  newAngle = magnitudeAngle-(acos((pow(magnitude,2)+pow(newMagnitude,2)-pow(mesuredDistance,2))/(2.0*magnitude*newMagnitude))* (180.0/PI));

  magnitude = newMagnitude;
  magnitudeAngle = newAngle;
  
  Serial.print("mesuredDistance : ");  //debug
  Serial.print(mesuredDistance);
  Serial.print("  | newMagnitude : ");
  Serial.print(newMagnitude);
  Serial.print("  | newAngle : ");
  Serial.print(newAngle);
  Serial.println("");
  
}

int readTOF(int numOfIterations){
  int succsesses = 0;
  int failures = 0;
  int data = 0;

  while(succsesses < numOfIterations && failures < numOfIterations){ // will run until it has enough data or failed mesurments
    
    delay(12);
    VL53L0X_RangingMeasurementData_t measure; // takes the actual mesurment
    
  
    TOF.rangingTest(&measure, false); // set it to 'true' to get all debug data in serial! (probably never be used)

    if (measure.RangeStatus != 4 && measure.RangeMilliMeter < 8000 ) {  // filter failures and incorrect data
    //Serial.println(measure.RangeMilliMeter); //prints distance
    data = data + measure.RangeMilliMeter;  
    succsesses++;

    } else {
    //Serial.println(" out of range ");
    failures++;
    }
  }

  if(failures < numOfIterations){
    data = data / numOfIterations; //converts mesurments into an avarige
    return(data);
  }else{
    return (0); //failed scan or out of range mesurments
  }
}

//work in progress

int sweep(int anglesToScan){ 
  // scans for the diabolo
  //returns the magnitude and angle of the diabolo location

  int anglesDone = 0;
  int distSum = 0; 
  int angleSum = 0;
  int hasMesured = 0;
  int numOfMesurment = 0;

  for(int i; i > anglesToScan; i++){
    int dist = readTOF(3);

    if (dist =! 0){
      distSum = distSum + dist;
      angleSum = angleSum + magnitudeAngle;
      numOfMesurment++;
    }
    stepperToAngle(magnitudeAngle++);
    delay(10);
  }
  distSum = distSum/numOfMesurment;
  angleSum = angleSum/numOfMesurment;

  return(angleSum,distSum);
}

void grab(){
  servoZ.write(170); //gripper lowers around object
  int gripAngle = 165;
int pressureValue;
  do{
    servoGrip.write(gripAngle--); //close gripper slightly
    
    pressureValue = analogRead(FSR_PIN); //read pressure sensor
    Serial.print("Pressure Value: ");
    Serial.println(pressureValue);

    if(gripAngle <= 45){
      break;
    }
  }while(pressureValue > thresholdPress); //check if gripper is gripping
}

