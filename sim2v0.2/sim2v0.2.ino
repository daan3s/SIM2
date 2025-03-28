



void loop() {

  dataIN();

  if (digitalRead(startButton) == LOW){
    delay(10);
    goToTarget(readTOF(10));
  }

    inverseK(magnitudeAngle,magnitude);

    baseServ.write(stepperAngle+magnitudeAngle);   //will be replaced with the stepper motor
    servoArm.write(180-servoArmAngle);    //i put my servo backards



    /*
    //debug
    Serial.print("stepperAngle : ");
    Serial.print(stepperAngle);
    Serial.print("  | servoArmAngle : ");
    Serial.print(servoArmAngle);
    Serial.print("  | magnitude : ");
    Serial.print(magnitude);
    Serial.print("  | magnitudeAngle : ");
    Serial.print(magnitudeAngle);
    Serial.println("");
    */
}






