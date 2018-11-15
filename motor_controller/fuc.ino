//enA and enB pins are PWM output pins
//in1, in2, in3, and in4 pins are digital outputs
//Motor1
//e1 controls the speed of motor1
int e1 = 10;
//pin m1 controls motor1 HIGH/LOW to set a direction
int m1 = 9;
//Motor2
//e2 controls the speed of motor2
int e2 = 6;
//pin m2 controls motor2 HIGH/LOW to set a direction
int m2 = 7;
void setup()
{
  // set all the motor control pins to outputs
  pinMode(e1, OUTPUT);
  pinMode(e2, OUTPUT);
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
}
//Turns motor1 backwards and motor2 forwards to turn left
void turnLeft(int speed)
{
  digitalWrite(m1, HIGH);
  analogWrite(e1, speed);
  digitalWrite(m2, LOW);
  analogWrite(e2, speed);
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
void moveForward(int speed) {
  digitalWrite(m1, HIGH);
  analogWrite(e1, speed);
  digitalWrite(m2, HIGH);
  analogWrite(e2, speed);
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
  turnRight(250,125);
  delay(2000);
  moveBackward(125,250);
  delay(2000);
}
