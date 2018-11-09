#include "Enes100.h"

#define abs(x) ((x)>0?(x):-(x))

//Make sure to update the third argument with the number of the plate 
Enes100 enes("TabascOSV", DEBRIS, 13, 2, 4);

boolean finished = false;

int period = 2000;
int timeCounter;

//enA and enB pins are PWM output pins
//in1, in2, in3, and in4 pins are digital outputs
//Motor1
//e1 controls the speed of motor1
int e1 = 10;
//pin m1 controls motor1 HIGH/LOW to set a direction
int m1 = 9;
//Motor2
//e2 controls the speed of motor2
int e2 = 5;
//pin m2 controls motor2 HIGH/LOW to set a direction
int m2 = 7;

void setup() {

  // set all the motor control pins to outputs
  pinMode(e1, OUTPUT);
  pinMode(e2, OUTPUT);
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);

  Serial.begin(9600);

  enes.println("Starting Navigation");

  while (!enes.retrieveDestination());
  while (!enes.updateLocation());

  timeCounter = millis() + period;

}

void loop() {
  //Turn to face objective
  fixAngle();

  timeCounter = millis() + period;
  while (!finished && millis() < timeCounter){
    
    //Moves forward until an object is detected or it reaches the endpoint
    while (!obstacleDetected() && ((abs(enes.location.x - enes.destination.x) > 0.15) || (abs(enes.location.y - enes.destination.y) > 0.15))){
      moveForward(255);
      while (!enes.updateLocation());
      Serial.println("Updated location"); 

      //Fixes the angle every two seconds
      if(millis() > timeCounter){
        Serial.println("PERIOD");
        break;
      }
    }
    while (!enes.updateLocation());

    //Checks for obstacles and endpoint
    if (readDistanceSensor(1) <= 350 || readDistanceSensor(2) <= 350){
      deactivateMotors();
      Serial.println("Obstacle detected");
      goAround();
    } else if ((abs(enes.location.x - enes.destination.x)) < 0.2 && (abs(enes.location.y - enes.destination.y)) < 0.2){
      finished = true;
      Serial.println("Found endpoint.");
      deactivateMotors();
    }
  }
  
  //Stops the program when finished
  if (finished){
    while(1);
  }
  
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
void turnRight(int speed)
{
  digitalWrite(m1, LOW);
  analogWrite(e1, speed);
  digitalWrite(m2, HIGH);
  analogWrite(e2, speed);
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
void moveBackward(int speed) {
  digitalWrite(m1, LOW);
  analogWrite(e1, speed);
  digitalWrite(m2, LOW);
  analogWrite(e2, speed);
}

/* Reads distance sensors. 
 * Each unit is worth approximately 5mm
 * The distance sensors are accurate to their farthest measurement (1024)
 * The distance sensors are accurate close range to the measurement 57.
 * This is about 250mm
 * The closer the object is to the sensor, the more inaccurate the measurement is
 * If an object is too close to a distance sensor, it will output a value in the 1000s
 * Tolerance of 10mm
*/
int readDistanceSensor(int d){
  int distance1, distance2;
  if(d == 1){
    //Reads from the called sensor, and then converts the measurement into millimeters
    distance1 = analogRead(A0) * 5;
    return distance1;
  }
  if(d == 2){
    //Reads from the called sensor, and then converts the measurement into millimeters
    distance2 = analogRead(A1) * 5;
    return distance2;
  }
}

boolean obstacleDetected(){
  if (readDistanceSensor(1) > 300 && readDistanceSensor(2) > 300){
    return false;
  }
  return true;
}

double determineTheta(double x, double y, double x2, double y2){
  double theta = asin((y2-y)/(sqrt((y2-y)*(y2-y)+(x2-x)*(x2-x))));
  return theta;
}

//Fixes the OSV heading to face the objective
void fixAngle(){

  Serial.println("Fixing angle");
  
  // Direction to turn. 0 = Right, 1 = Left
  int turnDirection = 0;
  while (!enes.updateLocation());
  double angle = determineTheta(enes.location.x, enes.location.y, enes.destination.x, enes.destination.y);
  
  if (enes.location.theta - angle <= 0){
    turnDirection = 1;
  }
  while (!enes.updateLocation());

  //Turns until within 0.05 units of the correct angle
  while (abs(enes.location.theta - determineTheta(enes.location.x, enes.location.y, enes.destination.x, enes.destination.y)) > 0.05) {

    if (turnDirection == 0){ //Right
      turnRight(255);  
    } else if (turnDirection == 1){ //Left
      turnLeft(255);
    }

    while (!enes.updateLocation());
    
  }
  Serial.println("Angle fixed");
}

//Obstacle avoidance algorithm
void goAround(){
  Serial.println("Going around obstacle");
  boolean avoided = false;
  deactivateMotors();
  int stuckCounter = millis() + 5000;
  
  while (!avoided){
    deactivateMotors();
    
    // Direction to turn as to avoid hitting the boundaries. 0 = left, 1 = right
    int turnDirection = 0;
    while (!enes.updateLocation());
    if (enes.location.y >= 1){
      turnDirection = 1;
    } 

    //Turns until there is no longer an obstacle in the OSV's path
    while (readDistanceSensor(1) <= 450 || readDistanceSensor(2) <= 450){
      if (turnDirection == 1){
        turnRight(255);
      } else {
        turnLeft(255);
      }
    }
    //tank.turnOffMotors();  

    timeCounter = millis() + 3000;
    while (millis() < timeCounter){
      moveForward(255);
    }
    //tank.turnOffMotors();
    
    fixAngle();
    
    if (!obstacleDetected()){
      avoided = true;
    }
  }
  
  Serial.println("Finished going around obstacle");
}