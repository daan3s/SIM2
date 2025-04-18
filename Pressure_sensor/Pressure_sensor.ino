#include <Servo.h>

#define FSR_PIN A0   // Analog pin for the FSR sensor
#define THRESHOLD 500 // Pressure threshold value

Servo myServo;

void setup() {
    Serial.begin(9600);
    myServo.attach(6); // Servo connected to pin "a"
    myServo.write(90); // Set servo to a neutral position
}

void loop() {
    

    if (pressureValue > THRESHOLD) {
        myServo.write(90); // Stop the servo (or hold position)
    } else {
        myServo.write(0);  // Move servo to another position
    }

    delay(100);
}
