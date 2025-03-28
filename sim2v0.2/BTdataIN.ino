
void dataIN(){
      //controll magnitude line lengh and its angle
  if (digitalRead(upButton) == LOW){
    magnitude =  magnitude + magnitudePercent*3; 
    delay(50);
  }
  if (digitalRead(downButton) == LOW){
    magnitude = magnitude - magnitudePercent*3;
    delay(50); 
  }
  if (digitalRead(rightButton) == LOW){
    magnitudeAngle = magnitudeAngle+2;
    delay(50); 
  }
  if (digitalRead(leftButton) == LOW){
    magnitudeAngle = magnitudeAngle-2;
    delay(50); 
  }

    //error avoidance(too big or too smal numbers)
  if(magnitude>stepperArmLengh+servoArmLengh){
    magnitude = stepperArmLengh+servoArmLengh;
  }
  if(magnitude<stepperArmLengh-servoArmLengh){
    magnitude = stepperArmLengh-servoArmLengh;
  }


}

