# include <Arduino.h>
# include <QTRSensors.h>
# include <SoftwareSerial.h>
int ok = 0;
// Pin Definitions - Motor Driver 1
const int driver1_ena = 44;  // Left Front Motor
const int driver1_in1 = 48;
const int driver1_in2 = 42;
const int driver1_in3 = 40;  // Right Front Motor
const int driver1_in4 = 43;
const int driver1_enb = 2;
// Pin Definitions - Motor Driver 2
const int driver2_ena = 45;  // Left Back Motor
const int driver2_in1 = 52;
const int driver2_in2 = 53;
const int driver2_in3 = 50;  // Right Back Motor
const int driver2_in4 = 51;
const int driver2_enb = 46;
const int emitterPin = 38;  // Enable the IR sensor
// Define Bluetooth module
SoftwareSerial BTSerial(19, 18);
// Speed Settings
const int BASE_SPEED = 130;
const int MAX_SPEED = 140;  //120
const int BACK_SPEED = 120;
const int OUTER_SENSOR_DELAY = 1300;
const int LINE_THRESHOLD = 700;  //700
static unsigned long lastTurnDelayTime = 0;  // Track time for delay
static bool isDelaying = false;             // Track if delay is active                        
bool isOnCenterline = false;               // Track center alignment
int P;
int I;
int D;
float  Kp = 0.07;
float Ki = 0.00001;
float Kd = 0;
int lastError = 0;
// Bluetooth Mode Handling
bool lineFollowingMode = true;  // Start in Line Following Mode
QTRSensors qtr;
const  uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
void calibrateSensors() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
 
  const int calibrationSpeed = 120;
  const int calibrationCycles = 2;
  const int samplesPerDirection = 25;
 
  delay(2000);
 
  for (int cycle = 0; cycle < calibrationCycles; cycle++) {
    for (int i = 0; i < samplesPerDirection; i++) {
      qtr.calibrate();
      setMotorSpeeds(calibrationSpeed, -calibrationSpeed);
      digitalWrite(LED_BUILTIN, i % 20 < 10);
      delay(20);
    }
   
    for (int i = 0; i < samplesPerDirection * 1.8; i++) {
      qtr.calibrate();
      setMotorSpeeds(-calibrationSpeed, calibrationSpeed);
      digitalWrite(LED_BUILTIN, i % 20 < 10);
      delay(20);
    }
   
    for (int i = 0; i < samplesPerDirection; i++) {
      qtr.calibrate();
      setMotorSpeeds(calibrationSpeed, -calibrationSpeed);
      digitalWrite(LED_BUILTIN, i % 20 < 10);
      delay(20);
    }
  }
 
  setMotorSpeeds(0, 0);
 
  for (int i = 0; i < 6; i++) {
    digitalWrite(LED_BUILTIN, i % 2);
    delay(50);
  }
  digitalWrite(LED_BUILTIN, LOW);
 
  delay(1000);
}
void setup()  {
 
  Serial1.begin(9600);
  qtr.setTypeRC();
  qtr.setSensorPins((const  uint8_t[]){A8, A9, A10, A11, A12, A13, A14, A15}, SensorCount);
  qtr.setEmitterPin(emitterPin);
  calibrateSensors();
  pinMode(driver1_ena, OUTPUT);
  pinMode(driver1_in1, OUTPUT);
  pinMode(driver1_in2, OUTPUT);
  pinMode(driver1_in3, OUTPUT);
  pinMode(driver1_in4, OUTPUT);
  pinMode(driver1_enb, OUTPUT);
  pinMode(driver2_ena, OUTPUT);
  pinMode(driver2_in1, OUTPUT);
  pinMode(driver2_in2, OUTPUT);
  pinMode(driver2_in3, OUTPUT);
  pinMode(driver2_in4, OUTPUT);
  pinMode(driver2_enb, OUTPUT);
  setMotorSpeeds(0, 0);
}
bool isStopSign() {
  int darkCount = 0;
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > LINE_THRESHOLD) {
      darkCount++;
    }
  }
  return darkCount >= 7;
}
bool isAllWhite() {
  int whiteCount = 0;
  for (uint8_t i = 0; i < SensorCount; i++) {
    whiteCount += (sensorValues[i] < LINE_THRESHOLD);
    if (whiteCount >= 6)  // Changed from < to >
      return true;
  }
  return false;
}
void checkForPause() {
  if (isStopSign()) {  // Check if the stop sign is detected
    unsigned long startTime = millis();
    while (millis() - startTime < 95) {  // Monitor for 200 ms
    qtr.readLineBlack(sensorValues);
      if (isAllWhite()) {  // If all sensors detect white
        setMotorSpeeds(-60, -60);
        delay(50);
        setMotorSpeeds(0,0);
        delay(5000);  // Pause for 2 seconds
        return;
      }
       
    }
    if (isStopSign()){
        while (!isAllWhite()) {
        setMotorSpeeds(BASE_SPEED, BASE_SPEED);           // Move forward at moderate speed
        qtr.readLineBlack(sensorValues);  // Keep reading sensors
      }
      delay(60);
      setMotorSpeeds(-255, -200);
      delay(70);
      setMotorSpeeds(0, 0);
      // Stop when all white is detected
      ok = 1;
      digitalWrite(LED_BUILTIN, HIGH);
   
    }
  }
}
void loop() {
    handleBluetoothCommands();
    if(lineFollowingMode)
    {
      if(ok == 1)
      {
        setMotorSpeeds(0,0);
        delay(10000);
      }
      checkForPause();
      PID_control();
    }
    else
    {
      handleRemoteControl();
    }
}
void PID_control() {
  uint16_t positionLine = qtr.readLineBlack(sensorValues);
  // Track center alignment
bool isOnCenterline = false;
// Check the outermost sensors for black detection
bool leftOuterSensorBlack = sensorValues[0] > 700;  // Leftmost sensor detects black
bool rightOuterSensorBlack = sensorValues[7] > 700; // Rightmost sensor detects black
// Determine if the car is on the centerline (Modify based on your center sensors)
bool centerLeftBlack = sensorValues[3] > 700;
bool centerRightBlack = sensorValues[4] > 700;
bool carOnCenterline = centerLeftBlack && centerRightBlack;
// If the car is on the centerline, update the flag
if (carOnCenterline) {
    isOnCenterline = true;
}
// **Trigger Delay & Turn Direction Based on Sensor Detection**
if (isOnCenterline && (leftOuterSensorBlack || rightOuterSensorBlack) && !isDelaying) {
    isDelaying = true;
    lastTurnDelayTime = millis();
    isOnCenterline = false;  // Reset flag until realigned
    // Decide turn direction
    if (leftOuterSensorBlack) {
        Serial1.println("Turning Right (Delay)");
    } else if (rightOuterSensorBlack) {
        Serial1.println("Turning Left (Delay)");
    }
}
// **During Delay: Apply Turning Logic**
if (isDelaying) {
    if (millis() - lastTurnDelayTime < OUTER_SENSOR_DELAY) {
        // Execute turn while delaying
        if (leftOuterSensorBlack && !rightOuterSensorBlack) {
            //Serial1.println("Turning right");
            setMotorSpeeds(BASE_SPEED, BASE_SPEED);
            delay(100);
            setMotorSpeeds(BACK_SPEED, -BACK_SPEED);  // Turn right
        } else if (rightOuterSensorBlack && !leftOuterSensorBlack) {
            setMotorSpeeds(BASE_SPEED, BASE_SPEED);
            delay(100);
            setMotorSpeeds(-BACK_SPEED, BASE_SPEED);  // Turn left
        }
        return;  // Maintain delay
        }
        else
        {
            isDelaying = false;
        }
}
// **Retrigger When Car Returns to Centerline**
if (!isDelaying && !carOnCenterline) {
    isOnCenterline = true; // Reset the flag to allow delay on next turn
}
  int error = 3500 - positionLine;
  P = error;
  I = error + I;
  D  = error - lastError;
  lastError = error;
  int motorSpeedChange = P*Kp  + I*Ki + D*Kd;
  int leftSpeed = BASE_SPEED + motorSpeedChange;
  int rightSpeed  = BASE_SPEED - motorSpeedChange;
  if (leftSpeed > MAX_SPEED) {
    leftSpeed =  MAX_SPEED;
  }
  if (rightSpeed > MAX_SPEED) {
    rightSpeed = MAX_SPEED;
  }
  if  (leftSpeed < -BACK_SPEED) {
    leftSpeed = -BACK_SPEED;
  }
  if (rightSpeed < -BACK_SPEED)  {
    rightSpeed = -BACK_SPEED;
  }
  setMotorSpeeds(leftSpeed, rightSpeed);
}
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Left motors direction
  if (leftSpeed >= 0) {
    digitalWrite(driver1_in1, HIGH);
    digitalWrite(driver1_in2, LOW);
    digitalWrite(driver2_in1, HIGH);
    digitalWrite(driver2_in2, LOW);
  } else {
    digitalWrite(driver1_in1, LOW);
    digitalWrite(driver1_in2, HIGH);
    digitalWrite(driver2_in1, LOW);
    digitalWrite(driver2_in2, HIGH);
    leftSpeed = -leftSpeed;
  }
  // Right motors direction
  if (rightSpeed >= 0) {
    digitalWrite(driver1_in3, LOW);
    digitalWrite(driver1_in4, HIGH);
    digitalWrite(driver2_in3, LOW);
    digitalWrite(driver2_in4, HIGH);
  } else {
    digitalWrite(driver1_in3, HIGH);
    digitalWrite(driver1_in4, LOW);
    digitalWrite(driver2_in3, HIGH);
    digitalWrite(driver2_in4, LOW);
    rightSpeed = -rightSpeed;
  }
  analogWrite(driver1_ena, leftSpeed);
  analogWrite(driver2_ena, leftSpeed);
  analogWrite(driver1_enb, rightSpeed);
  analogWrite(driver2_enb, rightSpeed);
}
void handleBluetoothCommands() {
  if (Serial1.available()) {
    char command = Serial1.read();
    //Serial.print("Command received: ");
    //Serial.println(command);
    switch (command) {
      case 'M':  // Switch to Line Following Mode
        lineFollowingMode = true;
        Serial1.println("Switched to Line Following Mode");
        break;
      case 'R':  // Switch to Remote Control Mode
        lineFollowingMode = false;
        setMotorSpeeds(0, 0);
        Serial1.println("Switched to Remote Control Mode");
        handleRemoteControl();
        break;
      case 'V':  // Switch to Remote Control Mode
        int ok = 0;
        setMotorSpeeds(0, 0);
        Serial1.println("Reset the stop");
        //handleRemoteControl();
        break;  
      default:
        Serial1.println("Invalid Command for Mode Switch");
        break;
    }
  }
}
void handleRemoteControl() {
  if (Serial1.available()) {  // Check for Bluetooth commands
    char command = Serial1.read();
    //Serial.print("Command received: ");
    //Serial.println(command);
    switch (command) {
      case 'F':  // Forward
        setMotorSpeeds(100, 100);
        //Serial.println("Moving Forward");
        break;
      case 'B':  // Backward
        setMotorSpeeds(-100, -100);
        //Serial.println("Moving Backward");
        break;
      case 'L':  // Left
        setMotorSpeeds(-120, 120);
        //Serial.println("Turning Left");
        break;
      case 'R':  // Right
        setMotorSpeeds(120, -120);
        //Serial.println("Turning Right");
        break;
      case 'S':  // Stop
        setMotorSpeeds(0, 0);
        //Serial.println("Stopped");
        break;
      default:
        //Serial.println("Unknown Command");
        break;
    }
  }
}