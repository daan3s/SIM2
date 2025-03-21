#include <SoftwareSerial.h>

#define DIR_PIN  4  
#define STEP_PIN 2  
#define EN_PIN   8  

const int STEPS_PER_REV = 200; // NEMA 17 full steps per revolution
const float STEPS_PER_DEGREE = STEPS_PER_REV / 360.0; // 200 / 360 = 5/9 steps per degree

float currentAngle = 0; // Tracks current motor position

SoftwareSerial BT(10, 11);  // Bluetooth RX, TX

// Mapping numbers to angles (15° steps, 25 → 345°)
const int numbers[] = {-1, 2, -9, 12, -15, 20, -21, 14, -17, 6, -7, 0, -3, 4, -11, 10, -13, 18, -22, 16, -19, 8, -5, 25};  
const int angles[]  = {0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180, 195, 210, 225, 240, 255, 270, 285, 300, 315, 330, 345};

void setup() {
    Serial.begin(9600);
    BT.begin(9600); // Bluetooth baud rate

    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);

    digitalWrite(EN_PIN, LOW);  // Enable motor driver

    Serial.println("Bluetooth Ready. Enter a number:");
    BT.println("Bluetooth Ready. Enter a number:");
}

void loop() {
    if (BT.available()) {
        String input = BT.readStringUntil('\n');  // Read input from Bluetooth
        input.trim();  

        if (input == "0d") {
            Serial.println("Command received: Returning to 0 degrees.");
            BT.println("Command received: Returning to 0 degrees.");
            moveToAngle(0);
            return;
        }

        int targetNumber = input.toInt(); // Convert to integer
        Serial.print("Received Bluetooth Command: ");
        Serial.println(targetNumber);

        int targetAngle = getAngleFromNumber(targetNumber);

        if (targetAngle != -1) {  // If valid number, move motor
            Serial.print("Target angle: ");
            Serial.println(targetAngle);
            moveToAngle(targetAngle);
        } else {
            Serial.println("Invalid input! Enter a valid number.");
            BT.println("Invalid input! Enter a valid number.");
        }
    }
}

int getAngleFromNumber(int num) {
    for (int i = 0; i < 24; i++) {
        if (numbers[i] == num) {
            return angles[i];
        }
    }
    return -1;  // Invalid number
}

void moveToAngle(float targetAngle) {
    float angleDifference = targetAngle - currentAngle;
    
    if (angleDifference == 0) { // If already at the position, do nothing
        Serial.println("Already at desired position.");
        BT.println("Already at desired position.");
        return;
    }

    int stepsToMove = abs(angleDifference) * STEPS_PER_DEGREE;  // Use correct step calculation for NEMA 17

    Serial.print("Moving ");
    Serial.print(stepsToMove);
    Serial.println(" steps");

    if (angleDifference > 0) {
        digitalWrite(DIR_PIN, LOW); // Reverse direction: CCW for increasing angle
    } else {
        digitalWrite(DIR_PIN, HIGH); // Reverse direction: CW for decreasing angle
    }

    for (int i = 0; i < stepsToMove; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(800);  // Adjust timing if needed
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(800);
    }

    currentAngle = targetAngle; // Update current position after moving
    Serial.println("Movement complete!");
    BT.println("Movement complete!");
}
