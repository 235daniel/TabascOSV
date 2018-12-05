#include "Enes100.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"

//Make sure to update the third argument with the number of the plate 
Enes100 enes("TabascOSV", DEBRIS, 25, 2, 4);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

boolean finishedNavigating = false;
boolean finishedMission = false;

const int period = 2000;
int timeCounter;

int rockyTerrainCounter = 1;
int materialAttempts = 0;

const double copper[3] = {107.5,85.8,61.3};
const double steel[3] = {82.1,94.9,74.8};
const double darkSteel[3] = {96.4,89.4,68.8};
const double air[3] = {101.4,88.7,59.4};
const double copperStDev = 4.5, steelStDev = 5.7, darkSteelStDev = 2.0, airStDev = 0.78;

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

void setup() {

  // set all the motor control pins to outputs
  pinMode(e1, OUTPUT);
  pinMode(e2, OUTPUT);
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);

  if (!tcs.begin()){
    enes.println("ERROR: Did not find color sensor");
    while(1);
  }

  Serial.begin(9600);

  enes.println("Starting Navigation");

  while (!enes.retrieveDestination());
  while (!enes.updateLocation());

  timeCounter = millis() + period;
  rockyTerrainCounter = 1;

}

void loop() {
  // Turn to face objective
  fixAngle();

  timeCounter = millis() + period;
  while (!finishedNavigating && millis() < timeCounter){
    
    // Moves forward until an object is detected or it reaches the endpoint
    moveForward(100);
    while (!obstacleDetected() && ((!myAbs(enes.location.x - enes.destination.x,0.275)) || (!myAbs(enes.location.y - enes.destination.y,0.275)))){
      while (!enes.updateLocation());
      enes.print("Distance Sensor 1: "); 
      enes.println(readDistanceSensor(1));

      enes.print("Distance Sensor 2: ");
      enes.println(readDistanceSensor(2));

      // Fixes the angle every two seconds
      if(millis() > timeCounter){
        enes.println("PERIOD");
        break;
      }

      //Stops the OSV briefly and gives it a chance to detect obstacles after leaving the rocky area.
      //Make sure to test if it's the rocky terrain area only, or the OSV cannot detect obstacles correctly anywhere 
      if (!isRockyTerrain && rockyTerrainCounter == 1){
        deactivateMotors();
        delay(1000);
        rockyTerrainCounter = 0;
        break;
      }
      
    }
    
    enes.println("Past initial loop, checking sensors/location");
    while (!enes.updateLocation());
    /*enes.print("x difference: ");
    enes.println(myAbs(enes.location.x - enes.destination.x,0.2));
    enes.print("y difference: "); 
    enes.println(myAbs(enes.location.y - enes.destination.y,0.2));
    enes.print("x location: ");
    enes.println(enes.location.x);*/

    // Checks for obstacles and endpoint
    if (obstacleDetected() && !isRockyTerrain() && !facingMaterial()){
      deactivateMotors();
      enes.println("Obstacle detected.");
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
  digitalWrite(m1, HIGH);
  analogWrite(e1, sped);
  digitalWrite(m2, LOW);
  analogWrite(e2, sped);
}
// Turns motor1 forwards and motor2 backwards to turn right
void turnRight(int sped)
{
  digitalWrite(m1, LOW);
  analogWrite(e1, sped);
  digitalWrite(m2, HIGH);
  analogWrite(e2, sped);
}
// Stops both motors
void deactivateMotors() {
  analogWrite(e1, 0);
  analogWrite(e2, 0);
}
// Moves both motors forward at a specified speed
void moveForward(int sped) {
  digitalWrite(m1, LOW);
  analogWrite(e1, sped);
  digitalWrite(m2, LOW);
  analogWrite(e2, sped);
}
// Moves both motors backwards at a specified speed
void moveBackward(int sped) {
  digitalWrite(m1, HIGH);
  analogWrite(e1, sped);
  digitalWrite(m2, HIGH);
  analogWrite(e2, sped);
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
  if (isRockyTerrain()){
    return false;
  }
  if (readDistanceSensor(1) > 450 && readDistanceSensor(2) > 700){
    return false;
  }
  return true;
}
boolean isRockyTerrain(){
  while (!enes.updateLocation());
  if (enes.location.x <= 1.25){
    return true;
  }
  return false;
}

//Distance sensors can see the material, so this should prevent the OSV from seeing it as an obstacle
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
double determineTheta(double x, double y, double x2, double y2){
  double theta = asin((y2-y)/(sqrt((y2-y)*(y2-y)+(x2-x)*(x2-x))));
  return theta;
}

// Fixes the OSV heading to face the objective
void fixAngle(){

  enes.println("Fixing angle");
  
  // Direction to turn. 0 = Right, 1 = Left
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
    
    // Direction to turn as to avoid hitting the boundaries. 0 = left, 1 = right
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

    timeCounter = millis() + 2000;
    while (millis() < timeCounter && !obstacleDetected()){
      moveForward(100);
    }

    deactivateMotors();
    
    fixAngle();
    
    if (!obstacleDetected()){
      avoided = true;
    }
  }
  
  enes.println("Finished going around obstacle");
}
boolean myAbs(double num, double range){
  return (num < range) && (num > -1.0*range);
}
boolean bottomQuadrant(){
  if (enes.destination.y > 1.0){
    return false;
  }
  return true;
}

//Color sensor
String getMaterialType(int r, int g, int b){
  double copperDist = sqrt(pow(copper[0] - r, 2) + pow(copper[1] - g, 2) + pow(copper[2] - b, 2));
  double steelDist = sqrt(pow(steel[0] - r, 2) + pow(steel[1] - g, 2) + pow(steel[2] - b, 2));
  double darkSteelDist = sqrt(pow(darkSteel[0] - r, 2) + pow(darkSteel[1] - g, 2) + pow(darkSteel[2] - b, 2));
  double airDist = sqrt(pow(air[0] - r, 2) + pow(air[1] - g, 2) + pow(air[2] - b, 2));

  if(copperDist < steelDist && copperDist < darkSteelDist){
    if(copperDist < 2*copperStDev){
      return "Copper";
    }
    return "No Material";
  }

  if(steelDist < 2*steelStDev){
    return "Steel";
  }
  
  if (darkSteelDist < 2*darkSteelStDev){
    return "Steel";
  }

  return "No Material";
}

// For mission objective 
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
void identifyMaterial(){
  
  int turnDirection = 0;
  while (!enes.updateLocation());
  double angle = determineTheta(enes.location.x, enes.location.y, enes.destination.x, enes.destination.y);
  
  if (enes.location.theta - angle <= 0){
    turnDirection = 1;
  }
  while (!enes.updateLocation());

  // Turns until within 0.15 units of the correct angle
  while (!myAbs(enes.location.theta - determineTheta(enes.location.x, enes.location.y, enes.destination.x, enes.destination.y), 0.15) && !finishedMission) {

    if (turnDirection == 0){ //Right
      turnRight(120);  
    } else if (turnDirection == 1){ //Left
      turnLeft(120);
    }

    while (!enes.updateLocation());
    readColorSensor();
    
  }
  if (!finishedMission){
    identifyMaterial();
  }
}
