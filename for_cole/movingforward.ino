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
//Enes100 enes("TabascOSV", DEBRIS, 7, 2, 4);
int turnAngle;

void setup()
{
  // set all the motor control pins to outputs
  pinMode(e1, OUTPUT);
  pinMode(e2, OUTPUT);
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  Serial.begin(9600);
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
void turnRight(int speed, int speed2)
{
  digitalWrite(m1, HIGH);
  analogWrite(e1, speed);
  digitalWrite(m2, HIGH);
  analogWrite(e2, speed2);
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
void moveBackward(int speed, int speed2) {
  digitalWrite(m1, LOW);
  analogWrite(e1, speed);
  digitalWrite(m2, LOW);
  analogWrite(e2, speed2);
}
void loop()
{  
  turnLeft(120);
}
