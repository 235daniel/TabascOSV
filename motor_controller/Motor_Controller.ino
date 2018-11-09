//enA and enB pins are PWM output pins
//in1, in2, in3, and in4 pins are digital outputs
//Motor1
//enA controls the speed of motor1
int enA = 10;
//pins in1 and in2 control motor1 one HIGH and one LOW to set a direction
int in1 = 9;
int in2 = 8;
//Motor2
//enB controls the speed of motor2
int enB = 5;
//pins in3 and in4 control motor2 one HIGH and one LOW to set a direction
int in3 = 7;
int in4 = 6;
void setup()
{
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}
//Turns motor1 backwards and motor2 forwards to turn left
void turnLeft(int speed)
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, speed);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, speed);
}
//Turns motor1 forwards and motor2 backwards to turn right
void turnRight(int speed)
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, speed);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, speed);
}
//Stops both motors
void deactivateMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
//Moves both motors forward at a specified speed
void moveForward(int speed) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, speed);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, speed);
}
//Moves both motors backwards at a specified speed
void moveBackward(int speed) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, speed);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, speed);
}
void loop()
{
  
}
