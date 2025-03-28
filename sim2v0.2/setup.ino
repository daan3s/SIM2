void setup() {
  // put your setup code here, to run once:
  pinMode(rightButton,INPUT_PULLUP);
  pinMode(upButton,INPUT_PULLUP);
  pinMode(downButton,INPUT_PULLUP);
  pinMode(leftButton,INPUT_PULLUP);
  pinMode(startButton,INPUT_PULLUP);

  servoArm.attach(10); //lower arm

  Serial.begin(9600);
  if (!TOF.begin()) {   //initialise serial for ToF
    Serial.println(F("Failed to boot Time of Flight sensor"));
    while(1);
  }
}

