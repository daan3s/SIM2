void stepperToAngle(float targetAngle) {
    float angleDifference = targetAngle - stepperAngle;
    float currentAngle = stepperAngle;

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