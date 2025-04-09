void inverseK(float ang,float mag)
{
  stepperAngle = (acos((pow(stepperArmLengh,2) + pow(mag,2) - pow(servoArmLengh,2)) / (2.0*stepperArmLengh*mag))) * (180.0 / PI); //upper arm (sloser to the base) angle to magnitude line
  servoArmAngle = (acos(((stepperArmLengh*stepperArmLengh) + (servoArmLengh*servoArmLengh) - (mag*mag)) / (2.0*stepperArmLengh*servoArmLengh))) * (180.0 / PI); //lower arm angle to uper arm
    
}

void goToTarget(float mesuredDistance){ 

  mesuredDistance = mesuredDistance+20 // adding 2 cm to align to the center of the diabolololo; 
  float newMagnitude;
  float newAngle;
  float sensAngle = abs(stepperAngle + servoArmAngle -90); //getting the angle between the magnitude line and the sensor direction 

  newMagnitude = sqrt(pow(magnitude,2)+pow(mesuredDistance,2)-2*mesuredDistance*magnitude*cos((sensAngle/180)* PI));
  newAngle = magnitudeAngle-(acos((pow(magnitude,2)+pow(newMagnitude,2)-pow(mesuredDistance,2))/(2.0*magnitude*newMagnitude))* (180.0/PI));

  magnitude = newMagnitude;
  magnitudeAngle = newAngle;
  
  Serial.print("mesuredDistance : ");  //debug
  Serial.print(mesuredDistance);
  Serial.print("  | newMagnitude : ");
  Serial.print(newMagnitude);
  Serial.print("  | newAngle : ");
  Serial.print(newAngle);
  Serial.println("");
  
}
