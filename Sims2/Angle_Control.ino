#define DIR_PIN  4   // Direction pin
#define STEP_PIN 2   // Step pin
#define EN_PIN   8   // Enable pin

const int STEPS_PER_REV = 200; // Change this if using microstepping
float currentAngle = 0; // Tracks the current angle

void setup() {
    Serial.begin(9600);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);

    digitalWrite(EN_PIN, LOW);  // Enable driver
    Serial.println("Enter an angle (e.g., 90, -45, 180):");
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');  // Read user input
        input.trim();  // Remove spaces and newline

        float targetAngle = input.toFloat(); // Convert input to float

        if (targetAngle != 0 || input == "0") { // Ensure valid input
            moveToAngle(targetAngle);
        } else {
            Serial.println("Invalid input! Please enter a number.");
        }
    }
}

void moveToAngle(float targetAngle) {
    float angleDifference = targetAngle - currentAngle;
    int stepsToMove = (angleDifference / 360.0) * STEPS_PER_REV;

    Serial.print("Moving to angle: ");
    Serial.println(targetAngle);

    if (stepsToMove > 0) {
        digitalWrite(DIR_PIN, HIGH); // CW direction
    } else {
        digitalWrite(DIR_PIN, LOW); // CCW direction
        stepsToMove = -stepsToMove; // Make steps positive
    }

    for (int i = 0; i < stepsToMove; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(1000);  // Adjust for speed
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(1000);
    }

    currentAngle = targetAngle; // Update the tracked angle
    Serial.println("Movement complete!");
}
