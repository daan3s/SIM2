void stepperToAngle(int targetAngle) {

    float angleDifference = targetAngle - currentStepperAngle;
    int stepsToMove = abs(angleDifference) * STEPS_PER_DEGREE;  

    Serial.print("Moving ");
    Serial.print(stepsToMove);
    Serial.println(" steps");

    digitalWrite(DIR_PIN, angleDifference > 0 ? HIGH : LOW);

    int minDelay = 5000; // higher valeue » Slow movement for high precision
    int maxDelay = 5500;  // if maxdelay is higher than mindelay gradual acceleration becomes gradual decelaretion  

    for (int i = 0; i < stepsToMove; i++) {
        int stepDelay = map(i, 0, stepsToMove, minDelay, maxDelay); // Gradual acceleration
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(stepDelay);
    }

    currentStepperAngle = targetAngle; 
    Serial.println("Movement complete!");
}
