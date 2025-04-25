#include <Servo.h>

// --- Stepper motor pins ---
#define DIR_PIN  4
#define STEP_PIN 2
#define EN_PIN   8

// --- Servo pins ---
#define SERVO_ARM_PIN 10    // Lower arm
#define SERVO_Z_PIN 13      // Z-axis servo
#define SERVO_GRIP_PIN 9    // Gripper servo

// --- Constants ---
const int STEPS_PER_REV = 1600;
const float STEPS_PER_DEGREE = STEPS_PER_REV / 360.0;
const int SERVO_SPEED = 15; // Degrees per second
int currentStepperAngle = 0;

Servo servoArm;   // Lower arm
Servo servoZ;     // Z-axis servo
Servo servoGrip;  // Gripper servo

// Number to angle mapping (24 positions, 15° apart)
const int numberAngles[][2] = {
  {2, 0},    {-1, 15},   {0, 30},    {-5, 45},
  {8, 60},   {-19, 75},  {16, 90},   {-22, 105},
  {18, 120}, {-13, 135}, {10, 150},  {-11, 165},
  {4, 180},  {-3, 195},  {0, 210},   {-7, 225},
  {6, 240},  {-17, 255}, {14, 270},  {-21, 285},
  {20, 300}, {-15, 315}, {12, 330},  {-9, 345}
};

// Function declarations
bool isValidNumber(String str);
int getAngleForNumber(int number);
void stepperToAngle(int targetAngle);
void moveServoSmoothly(Servo &servo, int targetAngle, int speedDegreesPerSec);
void handleBluetooth();

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  // Attach servos with specified pins
  servoArm.attach(SERVO_ARM_PIN);   // Lower arm
  servoZ.attach(SERVO_Z_PIN);       // Z-axis servo
  servoGrip.attach(SERVO_GRIP_PIN); // Gripper servo
  
  // Initialize all servos to their starting positions
  moveServoSmoothly(servoArm, 180, SERVO_SPEED);
  moveServoSmoothly(servoZ, 0, SERVO_SPEED);
  moveServoSmoothly(servoGrip, 0, SERVO_SPEED);
  
  Serial.println("System ready - All functions included");
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
        moveServoSmoothly(servoArm, 90, SERVO_SPEED);
        delay(500);
        moveServoSmoothly(servoZ, 0, SERVO_SPEED);
        delay(500);
        moveServoSmoothly(servoGrip, 180, SERVO_SPEED);
      } else {
        Serial.println("Input not found in mapping.");
      }
    } else {
      Serial.println("Invalid input.");
    }
  }
}

bool isValidNumber(String str) {
  if (str.length() == 0) return false;
  if (str.charAt(0) == '-') str = str.substring(1);
  for (unsigned int i = 0; i < str.length(); i++) {
    if (!isDigit(str[i])) return false;
  }
  return true;
}

int getAngleForNumber(int number) {
  for (int i = 0; i < 24; i++) {
    if (numberAngles[i][0] == number) {
      return numberAngles[i][1];
    }
  }
  return -1;
}

void stepperToAngle(int targetAngle) {
  float angleDifference = targetAngle - currentStepperAngle;
  int stepsToMove = abs(angleDifference) * STEPS_PER_DEGREE;

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

void moveServoSmoothly(Servo &servo, int targetAngle, int speedDegreesPerSec) {
  int currentAngle = servo.read();
  int step = (targetAngle > currentAngle) ? 1 : -1;
  int delayTime = 1000 / (speedDegreesPerSec / abs(step));
  
  while (currentAngle != targetAngle) {
    currentAngle += step;
    servo.write(currentAngle);
    delay(delayTime);
  }
}