#include <math.h>
#include <Servo.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X TOF = Adafruit_VL53L0X();    //declare time of flight


const int rightButton = 6;
const int upButton = 4;
const int downButton = 5;
const int leftButton = 3;
const int startButton = 7;

const float side1 = 228;  
const float side2 = 112;

int magnitude = map(51,0,100,side1-side2,side1+side2); //in millimeter
const float magnitudePercent = (side1-side2)/100;   //one percent between smallest and biggest magnitude
int angle = 90; // in deg

float angle1 = 0;   //upper arm's angle
float angle2 = 0;   //lower arm's angle

Servo baseServ; 
Servo endServ;


void setup() {
  // put your setup code here, to run once:
  pinMode(rightButton,INPUT_PULLUP);
  pinMode(upButton,INPUT_PULLUP);
  pinMode(downButton,INPUT_PULLUP);
  pinMode(leftButton,INPUT_PULLUP);
  pinMode(startButton,INPUT_PULLUP);

  baseServ.attach(11); //upper arm
  endServ.attach(10); //lower arm

  Serial.begin(9600);
  if (!TOF.begin()) {   //initialise serial for ToF
    Serial.println(F("Failed to boot Time of Flight sensor"));
    while(1);
  }
}


void loop() {

  //controll magnitude line lengh and its angle
if (digitalRead(upButton) == LOW){
  magnitude =  magnitude + magnitudePercent*3; 
  delay(50);
}
if (digitalRead(downButton) == LOW){
  magnitude = magnitude - magnitudePercent*3;
  delay(50); 
}
if (digitalRead(rightButton) == LOW){
  angle = angle+2;
  delay(50); 
}
if (digitalRead(leftButton) == LOW){
  angle = angle-2;
  delay(50); 
}

  //error avoidance(too big or too smal numbers)
if(magnitude>side1+side2){
  magnitude = side1+side2;
}
if(magnitude<side1-side2){
  magnitude = side1-side2;
}


if (digitalRead(startButton) == LOW){
  delay(10);
  goToTarget(readTOF(10));
}

  inverseK(angle,magnitude);

  baseServ.write(angle1+angle);   //will be replaced with the stepper motor
  endServ.write(180-angle2);    //i put my servo backards



  /*
  //debug
  Serial.print("angle1 : ");
  Serial.print(angle1);
  Serial.print("  | angle2 : ");
  Serial.print(angle2);
  Serial.print("  | magnitude : ");
  Serial.print(magnitude);
  Serial.print("  | angle : ");
  Serial.print(angle);
  Serial.println("");
  */
}

void inverseK(float ang,float mag)
{
  angle1 = (acos((pow(side1,2) + pow(mag,2) - pow(side2,2)) / (2.0*side1*mag))) * (180.0 / PI); //upper arm (sloser to the base) angle to magnitude line
  angle2 = (acos(((side1*side1) + (side2*side2) - (mag*mag)) / (2.0*side1*side2))) * (180.0 / PI); //lower arm angle to uper arm
    
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

void goToTarget(float mesuredDistance){ 

  mesuredDistance = mesuredDistance*1.08; //slightly incresed it to give it a satisfying smack
  float newMagnitude;
  float newAngle;
  float sensAngle = abs(angle1 + angle2 -90); //getting the angle between the magnitude line and the sensor direction 

  newMagnitude = sqrt(pow(magnitude,2)+pow(mesuredDistance,2)-2*mesuredDistance*magnitude*cos((sensAngle/180)* PI));
  newAngle = angle-(acos((pow(magnitude,2)+pow(newMagnitude,2)-pow(mesuredDistance,2))/(2.0*magnitude*newMagnitude))* (180.0/PI));

  magnitude = newMagnitude;
  angle = newAngle;
  
  Serial.print("mesuredDistance : ");  //debug
  Serial.print(mesuredDistance);
  Serial.print("  | newMagnitude : ");
  Serial.print(newMagnitude);
  Serial.print("  | newAngle : ");
  Serial.print(newAngle);
  Serial.println("");
  
}

void scanWMagnitude(){

}

void scanWAngle(int scanAngle){

  int scanned = 0;
  int dist = 0;
  int found = 0;
  int foundAng = 0;
  int foundDist = 0;

  while(scanned < scanAngle){
    baseServ.write(angle + scanned);   //will be replaced with the stepper motor
    delay(10);
    dist = readTOF(8);
    if(dist =! 0){
      found++;
      foundAng = foundAng + angle + scanned;
      foundDist = dist;
    }
    scanned++;
  }  
  return(foundAng/found,foundDist/found);

}