#include "Enes100Simulation.h"
#include "DFRTankSimulation.h"

#define abs(x) ((x)>0?(x):-(x))

Enes100Simulation enes;
DFRTankSimulation tank;
boolean finished = false;
int period = 2000;
int timeCounter;

void setup() {

  tank.init();

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
      tank.setLeftMotorPWM(255);
      tank.setRightMotorPWM(255);
      while (!enes.updateLocation());
      Serial.println("Updated location"); 

      //Fixes the angle every two seconds
      if(millis() > timeCounter){
        Serial.println("PERIOD");
        break;
      }
    }
    tank.turnOffMotors();
    while (!enes.updateLocation());

    //Checks for obstacles and endpoint
    if (enes.readDistanceSensor(0) <= 0.3 || enes.readDistanceSensor(2) <= 0.3){
      Serial.println("Obstacle detected");
      goAround();
    } else if ((abs(enes.location.x - enes.destination.x)) < 0.2 && (abs(enes.location.y - enes.destination.y)) < 0.2){
      finished = true;
      Serial.println("Found endpoint.");
      tank.turnOffMotors();
    }
  }
  
  //Stops the program when finished
  if (finished){
    while(1);
  }
  
}

boolean obstacleDetected(){
  if (enes.readDistanceSensor(0) > 0.25 && enes.readDistanceSensor(2) > 0.25){
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
      tank.setLeftMotorPWM(255);
      tank.setRightMotorPWM(-255);  
    } else if (turnDirection == 1){ //Left
      tank.setLeftMotorPWM(-255);
      tank.setRightMotorPWM(255);
    }

    while (!enes.updateLocation());
    
  }
  Serial.println("Angle fixed");
}

//Obstacle avoidance algorithm
void goAround(){
  Serial.println("Going around obstacle");
  boolean avoided = false;
  tank.turnOffMotors();
  
  while (!avoided){
    tank.turnOffMotors();
    
    // Direction to turn as to avoid hitting the boundaries. 0 = left, 1 = right
    int turnDirection = 0;
    while (!enes.updateLocation());
    if (enes.location.y >= 1){
      turnDirection = 1;
    } 

    //Turns until there is no longer an obstacle in the OSV's path
    while (enes.readDistanceSensor(0) <= 0.4 || enes.readDistanceSensor(2) <= 0.4){
      if (turnDirection == 1){
        tank.setLeftMotorPWM(255);
        tank.setRightMotorPWM(-255);
      } else {
        tank.setLeftMotorPWM(-255);
        tank.setRightMotorPWM(255);
      }
    }
    tank.turnOffMotors();  

    timeCounter = millis() + 3000;
    while (millis() < timeCounter){
      tank.setLeftMotorPWM(255);
      tank.setRightMotorPWM(255);
    }
    tank.turnOffMotors();
    fixAngle();
    
    if (!obstacleDetected()){
      avoided = true;
    }
  }
  
  Serial.println("Finished going around obstacle");
}
