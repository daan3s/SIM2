#define DIR_PIN  2 
#define STEP_PIN 3  
#define EN_PIN   4  

const int STEPS_PER_REV = 640000;  // 1/8 step mode
const float STEPS_PER_DEGREE = STEPS_PER_REV / 360.0;  // 4.44 steps per degree

float currentAngle = 0; 

const int validAngles[] = {0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 
                          180, 195, 210, 225, 240, 255, 270, 285, 300, 315, 330, 345, 360};

void setup() {
    Serial.begin(9600);

    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);

    digitalWrite(EN_PIN, LOW);  // Enable motor driver

    Serial.println("Stepper Motor Ready (1/8 Step Mode, Slow Motion). Enter an angle (0 to 360 in 15° steps):");
}

void loop() {
    if (Serial.available()) {
        

        int targetAngle = DataIN();
        //if (isValidAngle(targetAngle)) {
            Serial.print("Target angle: ");
            Serial.println(targetAngle);
            moveToAngle(targetAngle);
        //} else {
        //    Serial.println("Invalid angle! Enter a value from 0 to 360 in 15° steps.");
        //}
        delay(100); // Debounce input
    }
}

bool isNumber(String str) {
    if (str.length() == 0) return false;
    for (unsigned int i = 0; i < str.length(); i++) {
        if (!isDigit(str[i])) {
            return false;
        }
    }
    return true;
}

bool isValidAngle(int angle) {
    for (int i = 0; i < 25; i++) {
        if (validAngles[i] == angle) {
            return true;
        }
    }
    return false;
}

void moveToAngle(float targetAngle) {
    if (targetAngle == currentAngle) { 
        Serial.println("Already at desired position.");
        return;
    }

    float angleDifference = targetAngle - currentAngle;
    int stepsToMove = abs(angleDifference * STEPS_PER_DEGREE);  

    Serial.print("Moving ");
    Serial.print(stepsToMove);
    Serial.println(" steps");

    digitalWrite(DIR_PIN, angleDifference > 0 ? HIGH : LOW);


    for (int i = 0; i < stepsToMove; i++) {
        int stepDelay = 200;
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(stepDelay);
    }

    currentAngle = targetAngle; 
    Serial.println("Movement complete!");
}

int DataIN(){
  //need to make this not part advance until a command has been given
  int tenp;
  Serial.print("input a number pls : ");
  
  while(Serial.available() == 0){} //stops everything until there's input at serial 
    
  int input = Serial.parseInt();  
  serialFlush();
  
  Serial.println(input);
  return(input);

  
}

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}