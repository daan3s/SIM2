#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X TOF = Adafruit_VL53L0X();

void setup() {

  pinMode(3, INPUT_PULLUP); //button

  Serial.begin(9600);
  while (! Serial) {
    delay(1);
  }
  Serial.println("before tof");
  if (!TOF.begin()) {   //initialise serial for ToF
    Serial.println(F("Failed to boot Time of Flight sensor"));
    while(1);
  }
  Serial.println("after tof");
}

int somting;  // added for readability
void loop() {

  if(digitalRead(12) == LOW){ //reads when a button has been pressed
    somting = readTOF(10);    //the '10' is how many scans avarige it should return (for accuracy)
    Serial.println(somting);

  }
    
}

int readTOF(int numOfIterations){
  int succsesses = 0;
  int failures = 0;
  int data = 0;

  while(succsesses < numOfIterations && failures < numOfIterations){ // will run until it has enough data or failed mesurments
    
    delay(8);
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
