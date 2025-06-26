




int shortpin = 8;



    const int irEnablePin =  2;
    const int sensorPinArray[] = {A0,A1,A2,A3,A4,A5,A6,A7};     //order os sensor pins should be the same as the locaton of the sensors in reality (left to right)

  const int numofSensor = (sizeof(sensorPinArray)/sizeof(*sensorPinArray));     //calculates how many sensors are there
  bool sensorValeues[numofSensor];
  int frontThreshold[numofSensor];



void setup() {
  // put your setup code here, to run once:

    for(int i=0; i < numofSensor ;i++){     //set sensor pins as INPUT    
      pinMode(sensorPinArray[i], INPUT);
    }

  Serial.begin(9600);

pinMode (shortpin,INPUT);
pinMode(irEnablePin,OUTPUT);
delay(30);
digitalWrite(irEnablePin, HIGH);


delay(15);
initCALEBRATIONINATOR();

}

void loop() {
  // put your main code here, to run repeatedly:
  readSensors();

  for(int i = 0; i < numofSensor; i++){    //sums all sensor weights
        Serial.print("sens.");
          Serial.print(i);
          Serial.print(" : ");
          Serial.print(analogRead(sensorPinArray[i]));
        Serial.print("  |  ");


  }
Serial.print("     ||||     ");
  for(int i = 0; i < numofSensor; i++){    
        Serial.print("sens.");
          Serial.print(i);
          Serial.print("  :  ");
          Serial.print(sensorValeues[i]);
        Serial.print("  |  ");
  }
  Serial.println("");
  Serial.println("");

delay(200);

}

void readSensors(){     //reads the pins on sensorPinArray[] and writes down the results to sensorValeues[]

  for(int i = 0; i < numofSensor; i++){
    if(sensorPinArray[i] < PIN_A0){      //checks if selected pin is analog
      //pin is digital
      sensorValeues[i] = digitalRead(sensorPinArray[i]);     
    }else{      
      //pin is analog
      if(analogRead(sensorPinArray[i])<=frontThreshold[i]){     // does it surpasses threshold?
        sensorValeues[i] = 1;    
      }else{
        sensorValeues[i] = 0;
      }
    }
  } 
}




void initCALEBRATIONINATOR(){
  int lows[numofSensor];
  int highs[numofSensor];
  int thres = 0;

  Serial.println("put me on black line to start calibration");
  Serial.println("");
  Serial.print("waiting for pin ");
  Serial.print(shortpin);
  Serial.println(" to be shorted");

  while(true){
    if(digitalRead(shortpin) == HIGH){
      Serial.println("short");
      break;
    }
  }


  Serial.println("=======CHEKING LOW VALEUES BEGIN=======");
  

  for(int i = 0; i < numofSensor; i++){
    lows[i] = analogRead(sensorPinArray[i]);
  }
  Serial.println("=======CHEKING LOW VALEUES END=======");


  Serial.print("[2 seccond delay]");
  delay(2000);
  Serial.println("put me on white background to coninue calibration");
  Serial.print("waiting for pin ");
  Serial.print(shortpin);
  Serial.println(" to be shorted");

  while(true){
    if(digitalRead(shortpin) == HIGH){
      Serial.println("short");
      break;
    }
  }

  Serial.println("=======CHEKING HIGH VALEUES BEGIN=======");

  for(int i = 0; i < numofSensor; i++){
    highs[i] = analogRead(sensorPinArray[i]);
  }
  Serial.println("=======CHEKING HIGH VALEUES END=======");
  
  Serial.print("[1 seccond delay]");
  delay(1000);

  Serial.println("=======CREATING THRESHOLDS BEGIN=======");


  for(int i = 0; i < numofSensor; i++){
    thres = highs[i] - lows[i];
    thres = thres/2;
    frontThreshold[i] = (thres+lows[i]);
  }
  Serial.println("=======CREATING THRESHOLDS END=======");
  Serial.print("[2 seccond delay]");
  delay(2000);
  
}


























