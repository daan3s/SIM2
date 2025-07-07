

//########### presure sensor is disabled
//########### re enable it in the grab function
//########### once the pressure sensors are fixed/replaced


#include <math.h>
#include <Servo.h>
#include "Adafruit_VL53L0X.h"
#include <AccelStepper.h>


Adafruit_VL53L0X TOF = Adafruit_VL53L0X();    //declare time of flight

AccelStepper jimmy(AccelStepper::DRIVER,53,51); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
#define EN_PIN   52  
#define FSR_PIN A0

const float microStepMode = 4;//micro step is 4 actually
const float gearRatio=40.3;
const int degreeToSteps = microStepMode*gearRatio*(200.0/360.0);


const int numbers[] = {-1, 2, -9, 12, -15, 20, -21, 14, -17, 6, -7, 0, -3, 4, -11, 10, -13, 18, -22, 16, -19, 8, -5, 25};  
const int angles[]  = {0, -15, -30, -45, -60, -75, -90, -105, -120, -135, -150, -165, 180, 165, 150, 235, 120, 105, 90, 75, 60, 45, 30, 15};

const int thresholdPress = 512;

const float stepperArmLengh = 211;  
const float servoArmLengh = 129;

int magnitude = map(51,0,100,stepperArmLengh-servoArmLengh,stepperArmLengh+servoArmLengh); //in millimeter | starts at 50% extended

int magnitudeAngle = 0; // in deg

float stepperAngle = 0;   //upper arm's angle
float servoArmAngle = 0;   //lower arm's angle

bool debugMode=1;

Servo servoArm;
Servo servoZ;
Servo servoGrip;

void setup() {
  // put your setup code here, to run once:
  
  pinMode(FSR_PIN, INPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);  // Enable motor driver

  servoArm.attach(5); //lower arm 
  servoZ.attach(6); //temp pin for z servo
  servoGrip.attach(7);

  jimmy.setMaxSpeed(1500); //ALWAYS DECLARE THIS
  jimmy.setAcceleration(700); //ALWAYS DECLARE THIS
  jimmy.setCurrentPosition(0); //set current position to 0

  delay(2000);

  Serial.begin(9600);
  delay(100);
  Serial.println("bfore");
  if (!TOF.begin()) {   //initialise serial for ToF
    Serial.println(F("Failed to boot Time of Flight sensor"));
    //while(1);
  }
  Serial.println("after");

  servoZ.write(0);      //initial location of servos (good for debug)
  servoGrip.write(180);
  stepperToAngle(0);
  servoArm.write(0);

    Serial.println("~setup complete~");

/*
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
  */
}

int atAngle; // IF THIS WORKS IM ANGRY      im angry

void loop() {
  if (debugMode){
    //debugmode 
    //manualy execute functions
      Serial.println("list of functions :  1,inverseK   2,gototarget   3,readTOF  4,grab  5,sweep  6,steppertoangle  7,servoArm  8,Zservo  9,gripServo  10,ungrab  11,funSieres");
  switch(DataIN()) {
    case 1:
      //inverseK
      Serial.println("please input [target]magnitudeAngle (0-360) : ");
      magnitudeAngle = DataIN();
      Serial.println("please input [target]magnitude (0-100) : ");
      magnitude = magnitudePercent(DataIN());
      

      Serial.print("magnitudeAngle : ");
      Serial.println(magnitudeAngle);
      Serial.print("magnitude");
      Serial.println(magnitude);

      inverseK(magnitudeAngle,magnitude);

      Serial.print("stepper : ");
      Serial.print(magnitudeAngle - stepperAngle);
      Serial.print("  servo : ");
      Serial.println(servoArmAngle);
      
      stepperToAngle(magnitudeAngle - stepperAngle);
      servoArm.write(180-servoArmAngle);;
      break;

    case 2:
      //gototarget
      delay(5);
      Serial.println("please input angle where ToF mesured at : ");
      atAngle = DataIN();
      Serial.println("please input ToF mesurment (0-100) : ");
      goToTarget(atAngle,DataIN());
    break;
    

    case 3:
      //readTOF
      delay(5);

      Serial.println("please input number of iterations : ");
      Serial.println(readTOF(DataIN()));
    break;

    case 4:
      //grab
      delay(5);
      Serial.println("grabing...");
      grab();
    break;

    case 5:
      //sweep
      delay(5);
      Serial.println("enter amount of degrees to sweep through");
      sweep(DataIN());
    break;

    case 6:
      //steppertoangle
      delay(5);
      Serial.println("please input angle for stepper motor : ");
      stepperAngle = DataIN();
      stepperToAngle(stepperAngle);
      
    break;

    case 7:
      //servoArm
      delay(5);
      Serial.println("please input angle for servo : ");
      servoArmAngle = DataIN();
      servoArm.write(180-servoArmAngle);

    break;

    case 8:
      //Zservo
      delay(5);
      Serial.println("please input angle for servo : ");
      servoZ.write(DataIN());
      

    break;

    case 9:
      //gripServo
      delay(5);
      Serial.println("please input angle for servo : ");
      servoGrip.write(DataIN());

    break;

    case 10:
      //ungrab
      delay(5);
      Serial.println("ungrabing...");
      ungrab();
    break;

    case 11:
      //ungrab
      delay(5);
      Serial.println("comiting FUN");
      funSieres();
    break;
    

    default:
      Serial.println("command not recognised");
    }
    Serial.println("");
    Serial.println("");
  }else{
    //normal mode
    //automicly does everything
    // INCOMPLETE !!! 
    

      inverseK(magnitudeAngle,magnitude);

      stepperToAngle(magnitudeAngle - stepperAngle);   //will be replaced with the stepper motor
      servoArm.write(180-servoArmAngle);    //i put my servo backards

  }
}

int magnitudePercent(int perc){ //converts % to mm

  perc = map(perc,0,100,stepperArmLengh-servoArmLengh,stepperArmLengh+servoArmLengh);   //one percent between smallest and biggest magnitude
  return(perc);
}
int DataIN(){
  //need to make this not part advance until a command has been given
  int tenp;
  Serial.print("input a number pls : ");
  

  while(Serial.available() == 0){} //stops everything until there's input at serial 
    
  int input = Serial.parseInt();  
  serialFlush();
  
  Serial.println(input);
  return(input);

  
}

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}

void stepperToAngle(int targetAngle) {

    jimmy.moveTo(-targetAngle*degreeToSteps); //move 500 steps from 0 
    jimmy.runToPosition(); //Blocking, it will stop everything, go to the target position and then remove other things
 
    Serial.println("Movement complete!");
}

void inverseK(float ang,float mag)
{
  stepperAngle = (acos((pow(stepperArmLengh,2) + pow(mag,2) - pow(servoArmLengh,2)) / (2.0*stepperArmLengh*mag))) * (180.0 / PI); //upper arm (sloser to the base) angle to magnitude line
  servoArmAngle = (acos(((stepperArmLengh*stepperArmLengh) + (servoArmLengh*servoArmLengh) - (mag*mag)) / (2.0*stepperArmLengh*servoArmLengh))) * (180.0 / PI); //lower arm angle to uper arm
    
}

void goToTarget(int atAngle, float mesuredDistance){ 

  mesuredDistance;
  float newMagnitude;
  float AngleChange;
  float sensAngle = abs(stepperAngle + servoArmAngle -90); //getting the angle between the magnitude line and the sensor direction 

  newMagnitude = sqrt(pow(magnitude,2)+pow(mesuredDistance,2)-2*mesuredDistance*magnitude*cos((sensAngle/180)* PI));
  AngleChange = (acos((pow(magnitude,2)+pow(newMagnitude,2)-pow(mesuredDistance,2))/(2.0*magnitude*newMagnitude))* (180.0/PI));

  if(newMagnitude>stepperArmLengh+servoArmLengh){
    newMagnitude = stepperArmLengh+servoArmLengh;
  }

  magnitude = newMagnitude;
  magnitudeAngle = AngleChange + atAngle;
  

  Serial.print("mesuredDistance : ");  //debug
  Serial.print(mesuredDistance);
  Serial.print("  | magnitude : ");
  Serial.print(magnitude);
  Serial.print("  | magPecent : ");
  Serial.print(map(newMagnitude,stepperArmLengh-servoArmLengh,stepperArmLengh+servoArmLengh,0,100));
  Serial.print("  | magnitudeAngle : ");
  Serial.print(magnitudeAngle);
  Serial.print("  | AngleChange : ");
  Serial.print(AngleChange);
  Serial.println("");
  
}

int readTOF(int numOfIterations){
  int succsesses = 0;
  int failures = 0;
  int data = 0;

  while(succsesses < numOfIterations && failures < numOfIterations){ // will run until it has enough data or failed mesurments
    
   
    VL53L0X_RangingMeasurementData_t measure; // takes the actual mesurment
     delay(25);
      
    TOF.rangingTest(&measure, false); // set it to 'true' to get all debug data in serial! (probably never be used)

    if (measure.RangeStatus != 4 && measure.RangeMilliMeter < 900 && measure.RangeMilliMeter > 22) {  // filter failures and incorrect data
    Serial.println(measure.RangeMilliMeter); //prints distance
    data = data + measure.RangeMilliMeter;  
    succsesses++;

    } else {
    Serial.println(" out of range ");
    failures++;
    }
  }

  if(failures < numOfIterations){
    data = (data / (numOfIterations)); //converts mesurments into an avarige
    return(data);
  }else{
    return (0); //failed scan or out of range mesurments
  }
}

//work in progress

int sweepOut1;
int sweepOut2;

int sweep(int anglesToScan){ 
  // scans for the diabolo
  //returns the magnitude and angle of the diabolo location

  int distSum = 0; 
  int angleSum = 0;
  int hasMesured = 0;
  int numOfMesurment = 0;

  magnitude = magnitudePercent(83);
  inverseK(magnitudeAngle,magnitude);     //enter sweep position
  servoArm.write(180-servoArmAngle);;
  stepperToAngle(magnitudeAngle - stepperAngle);
  servoZ.write(180);
  servoGrip.write(0);

  int dist;
  int hasSeen = 0;
  angleSum = 0;
    jimmy.setAcceleration(1400); //ALWAYS DECLARE THIS

  for(int i = 0; i <= anglesToScan; i= i+2){
    
    stepperToAngle(magnitudeAngle - stepperAngle + i);

    delay(10);
    dist = readTOF(7);
    

    if (dist != 0){
      distSum = distSum + dist;
      angleSum = angleSum + (magnitudeAngle +i);
      numOfMesurment++;
    }

    if (numOfMesurment > 2){
      hasSeen++;
    }

    if(hasSeen && !dist || hasSeen > 20 || dist < 60 && dist > 4){
      break;
    }
  }

  jimmy.setAcceleration(700); //ALWAYS DECLARE THIS

  distSum = (distSum/numOfMesurment)+30;
  angleSum = (angleSum/numOfMesurment);

  Serial.print("saw object at ");
  Serial.print(angleSum);
  Serial.print(" at a distance of ");
  Serial.println(distSum);
  sweepOut1 = angleSum;
  sweepOut2 = distSum;
  return(angleSum,distSum);
}

void grab(){
  int gripAngle = 180;
  servoZ.write(165);
int pressureValue;
  do{
    servoGrip.write(gripAngle--); //close gripper slightly
    
    pressureValue = analogRead(FSR_PIN); //read pressure sensor
    //pressureValue = 1024; //remove this when re enebling pressure sensor
    Serial.print("Pressure Value: ");
    Serial.println(pressureValue);

    if(gripAngle <= 1){
      break;
    }
  }while(pressureValue < thresholdPress); //check if gripper is gripping
  servoZ.write(0); //gripper rises object
}

void ungrab(){
  servoZ.write(180);
  delay(700);
  servoGrip.write(180);
  delay(300);
  servoZ.write(0);
}

void funSieres(){
  delay(200);
  sweep(180);

  if(sweepOut2 <= 0){
    magnitudeAngle = 180;
    stepperToAngle(magnitudeAngle - stepperAngle);
    delay(150);

    sweep(180);
  }
  Serial.println("end fun sweep");
  goToTarget(sweepOut1,sweepOut2);
  delay(100);

  servoZ.write(0);
  delay(300);
  servoGrip.write(180);
  delay(300);
  

  inverseK(magnitudeAngle,magnitude);
  stepperToAngle(magnitudeAngle - stepperAngle);
  delay(150);
  servoArm.write(180-servoArmAngle);;
  delay(1600);

  grab();
  delay(150);

  magnitudeAngle = getAngleFromNumber(DataIN());
  magnitude = magnitudePercent(85);

  inverseK(magnitudeAngle,magnitude);
  stepperToAngle(magnitudeAngle - stepperAngle);
  delay(150);
  servoArm.write(180-servoArmAngle);;
  delay(600);

  ungrab();
}

int getAngleFromNumber(int num) {
    for (int i = 0; i < 24; i++) {
        if (numbers[i] == num) {
            return angles[i];
        }
    }
    return -1;  // Invalid number
}


