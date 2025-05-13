#include <math.h>
#include <Servo.h>

float side1 = 100;
float side2 = 85;

float angle1 = 0;
float angle2 = 0;

Servo baseServ;
Servo endServ;

int pot1;
int pot2;

void setup() {
  // put your setup code here, to run once:
  baseServ.attach(11);
  endServ.attach(10);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  pot1 = map(analogRead(A0), 0, 1023, 0, 180);
  pot2 = map(analogRead(A1), 0, 1023, side1-side2, side1+side2);
  
  inverseK(pot1,pot2);

  baseServ.write(angle1);
  endServ.write(angle2);

  delay(100);
}


int inverseK(float ang,float mag)
{
    angle1 = ang + (acos(((side1*side1)+(mag*mag)-(side2*side2))/(2.0*side1*mag)))* (180.0/3.141592653589793238463);
    angle2 = 180-(acos(((side1*side1)+(side2*side2)-(mag*mag))/(2.0*side1*side2)))* (180.0/3.141592653589793238463);
    
}