#include "Enes100Simulation.h"
#include "DFRTankSimulation.h"

#define abs(x) ((x)>0?(x):-(x))

Enes100Simulation enes;
DFRTankSimulation tank;
boolean finished = false;
int period = 2000;
int timeCounter = millis();

void setup() {

  tank.init();

  enes.println("Starting Navigation");

  while (!enes.retrieveDestination());

  while (!enes.updateLocation());

}

void loop() {
  //turn to face circle
  fixAngle();

  timeCounter = millis() + period;

  while (finished == false && millis() < timeCounter){
    while (!obstacleDetected() && millis() < timeCounter && ((abs(enes.location.x - enes.destination.x) > 0.2) || (abs(enes.location.y - enes.destination.y) > 0.2))){
      tank.setLeftMotorPWM(255);
      tank.setRightMotorPWM(255);
      while (!enes.updateLocation());      
    }
    tank.turnOffMotors();
    while (!enes.updateLocation());
    if (enes.readDistanceSensor(0) <= 0.3 || enes.readDistanceSensor(2) <= 0.3){
      Serial.println("Obstacle detected");
      goAround();
    } else if ((abs(enes.location.x - enes.destination.x)) < 0.25 && (abs(enes.location.y - enes.destination.y)) < 0.25){
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
  // Direction to turn. 0 = Right, 1 = Left
  int turnDirection = 0;
  while (!enes.updateLocation());
  double angle = determineTheta(enes.location.x, enes.location.y, enes.destination.x, enes.destination.y);
  
  if (enes.location.theta - angle <= 0){
    turnDirection = 1;
  } else if (enes.location.theta - angle > 0){
    turnDirection = 0;
  }
  
  while (abs(enes.location.theta - determineTheta(enes.location.x, enes.location.y, enes.destination.x, enes.destination.y)) > 0.05) {

  if (turnDirection = 0){
    tank.setLeftMotorPWM(255);
    tank.setRightMotorPWM(-255); 
  } else if (turnDirection = 1){
    tank.setLeftMotorPWM(-255);
    tank.setRightMotorPWM(255);
  }

  while (!enes.updateLocation());
    
  }
}

//Obstacle avoidance algorithm
void goAround(){
  boolean avoided = false;
  tank.turnOffMotors();
  while (!avoided){
    tank.turnOffMotors();
    // Direction to turn as to avoid hitting the boundaries. 0 = left, 1 = right
    int turnDirection = 0;
    while (!enes.updateLocation());
    if (enes.location.y - enes.destination.y >= 0){
      turnDirection = 1;
    } 
    
    while (enes.readDistanceSensor(0) <= 0.4 || enes.readDistanceSensor(2) <= 0.4){
      if (turnDirection = 1){
        tank.setLeftMotorPWM(255);
        tank.setRightMotorPWM(-255);
      } else {
        tank.setLeftMotorPWM(-255);
        tank.setRightMotorPWM(255);
      }
      while (!enes.updateLocation());
    }
    tank.turnOffMotors();  
    tank.setLeftMotorPWM(255);
    tank.setRightMotorPWM(255);

    timeCounter = millis() + period;
    while (millis() < timeCounter);
    
    tank.turnOffMotors();
    fixAngle();
    
    if (!obstacleDetected()){
      avoided = true;
    }
  }
  }
