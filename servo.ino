#include <Servo.h>

Servo upDownServo;   // Servo for up/down motion
Servo releaseServo;  // Servo for releasing the diabolo

const int upDownPin = 13; // First servo on pin 13
const int releasePin = 9; // Second servo on pin 9

void setup() {
  upDownServo.attach(upDownPin);
  releaseServo.attach(releasePin);

  // Start positions
  upDownServo.write(0);     // Start up
  releaseServo.write(0);  // Grip closed

  delay(1000); // Wait a second before starting
}

void loop() {
  // Move upDownServo from 0° to 180° (gripper goes down)
  for (int pos = 0; pos <= 180; pos++) {
    upDownServo.write(pos);
    delay(10);
  }

  delay(500); // Wait before releasing

  // Move releaseServo from 180° to 0° (release diabolo)
  for (int pos = 0; pos <= 180; pos++) {
    releaseServo.write(pos);
    delay(10);
  }

  // Done
  while (true);
}
