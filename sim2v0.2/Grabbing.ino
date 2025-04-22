void grab()
  servoZ.write(20); //gripper lowers around object
  int gripAngle = 90;
  int i = 0;
  do {
    servoGrip.write(gripAngle--); //close gripper slightly
    
    pressureValue = analogRead(FSR_PIN); //read pressure sensor
    Serial.print("Pressure Value: ");
    Serial.println(pressureValue);

    if(gripAngle <= 0){
      break;
    }
  }
  while(pressureValue == thresholdPress); //check if gripper is gripping