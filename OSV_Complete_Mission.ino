#include "Enes100.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"

//Make sure to update the third argument with the number of the plate 
Enes100 enes("TabascOSV", DEBRIS, 25, 2, 4);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

const int PERIOD = 2000;

// Average RGB values and standard deviations for copper, steel, and dark-colored steel
const double COPPER_AVG[3] = {107.5,85.8,61.3};
const double STEEL_AVG[3] = {82.1,94.9,74.8};
const double DARK_STEEL_AVG[3] = {96.4,89.4,68.8};
const double COPPER_STDEV = 4.5, STEEL_STDEV = 5.7, DARK_STEEL_STDEV = 2.0;

// Motor 1
// E1 controls the speed of motor 1
const int E1 = 10;
// pin M1 controls motor 1 HIGH/LOW to set a direction
const int M1 = 9;

// Motor 2
// E2 controls the speed of motor 2 
const int E2 = 6;
// pin M2 controls motor 2 HIGH/LOW to set a direction
const int M2 = 7;

boolean finishedNavigating = false;
boolean finishedMission = false;
int timeCounter;

void setup() {

  // Set all the motor control pins to outputs
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  if (!tcs.begin()){
    enes.println("ERROR: Did not find color sensor");
    while(1);
  }

  Serial.begin(9600);

  enes.println("Starting Navigation");

  while (!enes.retrieveDestination());
  while (!enes.updateLocation());

  timeCounter = millis() + PERIOD;

}

void loop() {
  // Turn to face objective
  fixAngle();

  timeCounter = millis() + PERIOD;
  while (!finishedNavigating && millis() < timeCounter){
    
    // Moves forward until an object is detected or it reaches the endpoint
    moveForward(100);
    
    while (!obstacleDetected() && ((!myAbs(enes.location.x - enes.destination.x,0.275)) || (!myAbs(enes.location.y - enes.destination.y,0.275)))){
      while (!enes.updateLocation());

      // Fixes the angle every two seconds
      if(millis() > timeCounter){
        enes.println("Counter has ended, attempting to fix the angle");
        break;
      }      
    }
    
    enes.println("Past initial loop, checking sensors/location");
    while (!enes.updateLocation());

    // Checks for obstacles and endpoint. Obstacle detection is ignored if the OSV is in the rocky terrain.
    if (obstacleDetected() && !isRockyTerrain() && !facingMaterial()){
      deactivateMotors();
      enes.println("Obstacle detected.");
      
      // Moves backwards a small distance, since the sensors don't detect the obstacle until the OSV is very close to it.
      moveBackward(100);
      delay(800);
      deactivateMotors();
      goAround();
      
    } else if (myAbs(enes.location.x - enes.destination.x,0.275) && (myAbs(enes.location.y - enes.destination.y,0.275)) && !isRockyTerrain()){
      finishedNavigating = true;
      enes.println("Found endpoint.");
      enes.navigated(); 
      deactivateMotors();
    }
  }

  // Checks the color sensor when finished navigating
  if (finishedNavigating){
    fixAngle();
    
    // Moves a small distance forward if not close enough to the material
    if (readDistanceSensor(1) >= 450 && readDistanceSensor(2) >= 700 && facingMaterial()){
      timeCounter = millis() + 100;
      moveForward(100);
      while (readDistanceSensor(1) >= 450 && readDistanceSensor(2) >= 700 && facingMaterial() && timeCounter > millis());
    }
    deactivateMotors();
    identifyMaterial();
  }
  
  // Stops the program when finished
  if (finishedMission){
    enes.endMission();
    deactivateMotors();
    while(1);
  }

  
}

// Turns motor1 backwards and motor2 forwards to turn left
void turnLeft(int sped)
{
  digitalWrite(M1, HIGH);
  analogWrite(E1, sped);
  digitalWrite(M2, LOW);
  analogWrite(E2, sped);
}
// Turns motor1 forwards and motor2 backwards to turn right
void turnRight(int sped)
{
  digitalWrite(M1, LOW);
  analogWrite(E1, sped);
  digitalWrite(M2, HIGH);
  analogWrite(E2, sped);
}
// Stops both motors
void deactivateMotors() {
  analogWrite(E1, 0);
  analogWrite(E2, 0);
}
// Moves both motors forward at a specified speed
void moveForward(int sped) {
  digitalWrite(M1, LOW);
  analogWrite(E1, sped);
  digitalWrite(M2, LOW);
  analogWrite(E2, sped);
}
// Moves both motors backwards at a specified speed
void moveBackward(int sped) {
  digitalWrite(M1, HIGH);
  analogWrite(E1, sped);
  digitalWrite(M2, HIGH);
  analogWrite(E2, sped);
}

/* Reads distance sensors. 
 * Each unit is worth approximately 5mm
 * The distance sensors are accurate to their farthest measurement (1024)
 * The distance sensors are accurate close range to the measurement 57.
 * This is about 250mm
 * The closer the object is to the sensor, the more inaccurate the measurement is
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

// Checks distance sensors to determine if there is an obstacle. 
boolean obstacleDetected(){
  
  // Ignores sensors if the OSV is in the rocky terrain
  if (isRockyTerrain()){
    return false;
  }
  
  if (readDistanceSensor(1) > 450 && readDistanceSensor(2) > 700){
    return false;
  }
  return true;
}

// Checks if the OSV is in the rocky terrain, which ends at the x value 1.25
boolean isRockyTerrain(){
  while (!enes.updateLocation());
  if (enes.location.x <= 1.25){
    return true;
  }
  return false;
}

// Checks if the OSV is facing the material so that the OSV does not try to avoid it. 
boolean facingMaterial(){
  while (!enes.updateLocation());
  double angle = determineTheta(enes.location.x, enes.location.y, enes.destination.x, enes.destination.y);
  if (bottomQuadrant()){
    if (enes.location.x >= 2.15 && enes.location.y <= 1.0 && myAbs(enes.location.theta - angle, 0.2)){
      return true;
    }
  } else if (enes.location.x >= 2.15 && enes.location.y > 1.0 && myAbs(enes.location.theta - angle, 0.2)) {
    return true;
  }
  enes.println("Is not facing material");
  return false;
}

// Determines the angle that the OSV needs to turn to 
double determineTheta(double x, double y, double x2, double y2){
  double theta = asin((y2-y)/(sqrt((y2-y)*(y2-y)+(x2-x)*(x2-x))));
  return theta;
}

// Fixes the OSV heading to face the objective
void fixAngle(){

  enes.println("Fixing angle");
  
  // Calculates the most efficient direction to turn. 0 = Right, 1 = Left
  int turnDirection = 0;
  while (!enes.updateLocation());
  double angle = determineTheta(enes.location.x, enes.location.y, enes.destination.x, enes.destination.y);
  
  if (enes.location.theta - angle <= 0){
    turnDirection = 1;
  }
  
  while (!enes.updateLocation());

  // Turns until within 0.2 units of the correct angle
  while (!myAbs(enes.location.theta - determineTheta(enes.location.x, enes.location.y, enes.destination.x, enes.destination.y), 0.2)) {

    if (turnDirection == 0){ //Right
      turnRight(120);  
    } else if (turnDirection == 1){ //Left
      turnLeft(120);
    }

    while (!enes.updateLocation());
    
  }
  enes.println("Angle fixed");
}

// Obstacle avoidance algorithm
void goAround(){
  enes.println("Going around obstacle");
  boolean avoided = false;
  
  while (!avoided){
    deactivateMotors();
    
    // Calculates the direction to turn to avoid hitting the boundaries. 0 = left, 1 = right
    int turnDirection = 0;
    while (!enes.updateLocation());
    if (enes.location.y >= 1){
      turnDirection = 1;
    } 

    // Turns until there is no longer an obstacle in the OSV's path
    while (readDistanceSensor(1) <= 450 || readDistanceSensor(2) <= 700){
      enes.print("Distance Sensor 1: "); 
      enes.println(readDistanceSensor(1));

      enes.print("Distance Sensor 2: ");
      enes.println(readDistanceSensor(2));
      if (turnDirection == 1){
        turnRight(120);
      } else {
        turnLeft(120);
      }
    } 

    // After finding a clear path, moves for a short amount of time
    timeCounter = millis() + 2000;
    while (millis() < timeCounter && !obstacleDetected()){
      moveForward(100);
    }

    deactivateMotors();
    
    // Readjusts the OSV to face the objective. 
    fixAngle();
    
    // If an obstacle is still in the way, it will try again
    if (!obstacleDetected()){
      avoided = true;
    }
  }
  
  enes.println("Finished going around obstacle");
}

// Custom absolute value function since the default abs() function is not able to return a double.
boolean myAbs(double num, double range){
  return (num < range) && (num > -1.0*range);
}

// Cheks if the destination is in the bottom (4th) quadrant or not.
boolean bottomQuadrant(){
  if (enes.destination.y > 1.0){
    return false;
  }
  return true;
}

// Calculates the material type based on the r, g, and b values using the distance formula 
String getMaterialType(int r, int g, int b){
  double copperDist = sqrt(pow(COPPER_AVG[0] - r, 2) + pow(COPPER_AVG[1] - g, 2) + pow(COPPER_AVG[2] - b, 2));
  double steelDist = sqrt(pow(STEEL_AVG[0] - r, 2) + pow(STEEL_AVG[1] - g, 2) + pow(STEEL_AVG[2] - b, 2));
  double darkSteelDist = sqrt(pow(DARK_STEEL_AVG[0] - r, 2) + pow(DARK_STEEL_AVG[1] - g, 2) + pow(DARK_STEEL_AVG[2] - b, 2));

  if(copperDist < steelDist && copperDist < darkSteelDist){
    if(copperDist < 2*COPPER_STDEV){
      return "Copper";
    }
    return "No Material";
  }

  if(steelDist < 2*STEEL_STDEV){
    return "Steel";
  }
  
  if (darkSteelDist < 2*DARK_STEEL_STDEV){
    return "Steel";
  }

  return "No Material";
}

// Reads the color sensor and ends the mission if copper or steel is detected.
void readColorSensor(){
  enes.println("Reading color sensor..");
  uint16_t clear, red, green, blue;
  tcs.setInterrupt(false); // Turn on LED
  
  delay(60);
  
  tcs.getRawData(&red, &green, &blue, &clear);
  
  tcs.setInterrupt(true); // Turn off LED

  uint32_t sum = clear;
  float r, g, b;
  
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;
  
  String material = getMaterialType((int)r, (int)g, (int)b);

  if (material == "Copper"){
    enes.baseObjective(COPPER);
    finishedMission = true;
    deactivateMotors();
    enes.endMission();
  } else if (material == "Steel"){
    enes.baseObjective(STEEL);
    finishedMission = true;
    deactivateMotors();
    enes.endMission();
  }
}

// Algorithm for measuring and transmitting the type of material
void identifyMaterial(){
 
  // Calculates the most efficient turn direction
  int turnDirection = 0;
  while (!enes.updateLocation());
  double angle = determineTheta(enes.location.x, enes.location.y, enes.destination.x, enes.destination.y);
  
  if (enes.location.theta - angle <= 0){
    turnDirection = 1;
  }
  while (!enes.updateLocation());

  // Turns until within 0.15 units of the correct angle towards the material
  while (!myAbs(enes.location.theta - determineTheta(enes.location.x, enes.location.y, enes.destination.x, enes.destination.y), 0.15) && !finishedMission) {

    if (turnDirection == 0){ //Right
      turnRight(120);  
    } else if (turnDirection == 1){ //Left
      turnLeft(120);
    }

    while (!enes.updateLocation());
    
    // Continuously reads the color sensor while turning
    readColorSensor();
    
  }
  // Tries again if the color sensor did not detect copper or steel and end the mission
  if (!finishedMission){
    identifyMaterial();
  }
}
