void setup() {
  // put your setup code here, to run once:
  pinMode(rightButton,INPUT_PULLUP);
  pinMode(upButton,INPUT_PULLUP);
  pinMode(downButton,INPUT_PULLUP);
  pinMode(leftButton,INPUT_PULLUP);
  pinMode(startButton,INPUT_PULLUP);

  pinMode(DIR_PIN, OUTPUT);  //stepper pinout
  pinMode(STEP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);  // Enable motor driver

  servoArm.attach(10); //lower arm
  servoZ.attach(13); //temp pin for z servo
  servoGrip.attach(9);

  Serial.begin(9600);
  if (!TOF.begin()) {   //initialise serial for ToF
    Serial.println(F("Failed to boot Time of Flight sensor"));
    while(1);
  }
  //grab();

  Serial.println("start in debug mode?   (type 1 for yes)")
  switch(DataIN()) {
    case 1:
      debugMode = 1;
      Serial.println("debug mode !");
      break;
    default:
      debugMode = 0;
      Serial.println("normal mode !");
  }
}

