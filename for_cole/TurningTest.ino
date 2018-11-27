#define abs(x)  ( (x) > 0 ? (x) : -(x) )
#include "Enes100.h"

//enA and enB pins are PWM output pins
//in1, in2, in3, and in4 pins are digital outputs
//Motor1
//e1 controls the speed of motor1
const int e1 = 10;
//pin m1 controls motor1 HIGH/LOW to set a direction
const int m1 = 9;
//Motor2
//e2 controls the speed of motor2
const int e2 = 6;
//pin m2 controls motor2 HIGH/LOW to set a direction
const int m2 = 7;
Enes100 enes("TabascOSV", DEBRIS, 7, 2, 4);
double turnAngle;

void setup()
{
  // set all the motor control pins to outputs
  pinMode(e1, OUTPUT);
  pinMode(e2, OUTPUT);
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  Serial.begin(9600);
  while (!enes.updateLocation());
  enes.println("Updated Location Initially");
}
//Turns motor1 backwards and motor2 forwards to turn left
void turnLeft(int sped)
{
  digitalWrite(m1, HIGH);
  analogWrite(e1, sped);
  digitalWrite(m2, LOW);
  analogWrite(e2, sped);
}
//Turns motor1 forwards and motor2 backwards to turn right
void turnRight(int sped)
{
  digitalWrite(m1, LOW);
  analogWrite(e1, sped);
  digitalWrite(m2, HIGH);
  analogWrite(e2, sped);
}
//Stops both motors
void deactivateMotors() {
  analogWrite(e1, 0);
  analogWrite(e2, 0);
}
//Moves both motors forward at a specified speed
void moveForward(int sped) {
  digitalWrite(m1, HIGH);
  analogWrite(e1, sped);
  digitalWrite(m2, HIGH);
  analogWrite(e2, sped);
}
//Moves both motors backwards at a specified speed
void moveBackward(int sped, int speed2) {
  digitalWrite(m1, LOW);
  analogWrite(e1, sped);
  digitalWrite(m2, LOW);
  analogWrite(e2, speed2);
}
void loop()
{ 
  unsigned long lastTime; 
  delay(2000);
  while(!enes.updateLocation());
  enes.print("Initial angle = ");
  enes.println(enes.location.theta);
  turnAngle = enes.location.theta + 3.1415/2.0;
  if (turnAngle > 3.1415)
    turnAngle -= 2*3.1415;
  enes.print("Goal angle = ");
  enes.println(turnAngle);
  turnLeft(120);
  lastTime = millis();
  while ((turnAngle - enes.location.theta) > 0.2 || (turnAngle - enes.location.theta) < -.2 ){
    
    enes.print("just subtract: ");
    enes.println((turnAngle - enes.location.theta));
    
    while (!enes.updateLocation());
    if ((millis() - lastTime) > 100) {
      enes.print("Current angle = ");    
      enes.println(enes.location.theta);    
      lastTime = millis();
    }
  }
  deactivateMotors();
  enes.print("Final angle = ");    
  enes.println(enes.location.theta);    
}
