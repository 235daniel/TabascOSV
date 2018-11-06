void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}
/* Reads distance sensors. 
 * Each unit is worth approximately 5mm
 * The distance sensors are accurate to their farthest measurement (1024)
 * The distance sensors are accurate close range to the measurement 57.
 * This is about 250mm
 * The closer the object is to the sensor, the more inaccurate the measurement is
 * If an object is too close to a distance sensor, it will output a valeu in the 1000s
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
void loop() {
  Serial.println(readDistanceSensor(1));
  readDistanceSensor(2);
  delay(500);
}
