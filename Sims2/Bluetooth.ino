#include <Servo.h>

// --- Stepper motor pins ---
#define DIR_PIN  4
#define STEP_PIN 2
#define EN_PIN   8

// --- Servo pin ---
#define SERVO_PIN 10

// --- Constants ---
const int STEPS_PER_REV = 1600;
const float STEPS_PER_DEGREE = STEPS_PER_REV / 360.0;
int currentStepperAngle = 0;

Servo servoArm;

// Number to angle mapping (24 positions, 15° apart)
const int numberAngles[][2] = {
  {2, 0},    {-1, 15},   {0, 30},    {-5, 45},
  {8, 60},   {-19, 75},  {16, 90},   {-22, 105},
  {18, 120}, {-13, 135}, {10, 150},  {-11, 165},
  {4, 180},  {-3, 195},  {100, 210},   {-7, 225},
  {6, 240},  {-17, 255}, {14, 270},  {-21, 285},
  {20, 300}, {-15, 315}, {12, 330},  {-9, 345}
};

void setup() {
  Serial.begin(9600);      // USB Serial for debugging
  Serial1.begin(9600);     // Bluetooth on Serial1 (RX1=19, TX1=18)

  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  servoArm.attach(SERVO_PIN);
  servoArm.write(180);     // Start at 180°

  Serial.println("System ready - Exact angle mapping");
}

void loop() {
  handleBluetooth();
}

void handleBluetooth() {
  if (Serial1.available()) {
    String inputStr = Serial1.readStringUntil('\n');
    inputStr.trim();

    if (isValidNumber(inputStr)) {
      int inputNum = inputStr.toInt();
      int targetAngle = getAngleForNumber(inputNum);

      if (targetAngle >= 0) {
        Serial.print("Received: ");
        Serial.print(inputNum);
        Serial.print(" → Angle: ");
        Serial.println(targetAngle);

        stepperToAngle(targetAngle);
        servoArm.write(90);  // Move to 90° after stepper completes
      } else {
        Serial.println("Input not found in mapping.");
      }
    } else {
      Serial.println("Invalid input.");
    }
  }
}

int getAngleForNumber(int number) {
  for (int i = 0; i < 24; i++) {
    if (numberAngles[i][0] == number) {
      return numberAngles[i][1];
    }
  }
  return -1;
}

bool isValidNumber(String str) {
  if (str.length() == 0) return false;
  if (str.charAt(0) == '-') str = str.substring(1);
  for (unsigned int i = 0; i < str.length(); i++) {
    if (!isDigit(str[i])) return false;
  }
  return true;
}

void stepperToAngle(int targetAngle) {
  float angleDifference = targetAngle - currentStepperAngle;
  int stepsToMove = abs(angleDifference) * STEPS_PER_DEGREE;

  // Set direction based on shortest path
  digitalWrite(DIR_PIN, angleDifference > 0 ? HIGH : LOW);

  for (int i = 0; i < stepsToMove; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(5000);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(5000);
  }

  currentStepperAngle = targetAngle;
  Serial.println("Stepper movement complete.");
}