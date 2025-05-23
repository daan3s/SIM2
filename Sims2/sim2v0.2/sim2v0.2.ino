



void loop() {
  if (debugmode){
    //debugmode 
    //manualy execute functions
      Serial.println("list of functions :  1,inverseK   2,gototarget   3,readTOF  4,grab  5,sweep  6,steppertoangle  7,servoArm");
      Serial.println("please input the number next to the function to select it");
  switch(DataIN()) {
    case 1:
      //inverseK
      Serial.println("please input [target]magnitudeAngle (0-360) : ");
      magnitudeAngle = DataIN();
      Serial.println("please input [target]magnitude (0-100) : ");
      magnitude = DataIN()*magnitudePercent;
      inverseK(magnitudeAngle,magnitude);
      break;
    case 2:
      //gototarget
      Serial.println("please input [target]magnitude (0-100) : ");
      gototarget(DataIN());
      
    break;

    case 3:
      //readTOF
      Serial.println("please input number of iterations : ");
      Serial.println(readTOF(DataIN()));
    break;

    case 4:
      //grab
      Serial.println("grabing...");
      grab();
    break;

    case 5:
      //sweep
      Serial.println("incomplete");
    break;

    case 6:
      //steppertoangle
      Serial.println("please input angle for stepper motor : ");
      stepperAngle = DataIN();
      steppertoangle(stepperAngle)
      
    break;

    case 7:
      //servoArm
      Serial.println("please input angle for stepper motor : ");
      servoArmAngle = DataIN();
      servoArm.write(servoArmAngle);    //i put my servo backards

    break;

    default:
      Serial.println("command not recognised");
    }
  }else{
    //normal mode
    //automicly does everything
    // INCOMPLETE !!! 
    dataIN();

    if (digitalRead(startButton) == LOW){
      delay(10);
      goToTarget(readTOF(10));
    }

      inverseK(magnitudeAngle,magnitude);

      stepperToAngle(stepperAngle+magnitudeAngle);   //will be replaced with the stepper motor
      servoArm.write(servoArmAngle);    //i put my servo backards


  }
}






