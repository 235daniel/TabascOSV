#include "Enes100.h"
#include "DFRTank.h"
#include <SoftwareSerial.h>
#include <PololuQik.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

#define abs(x) ((x)>0?(x):-(x))

//Make sure to update the third argument with the number of the plate 
Enes100 enes("TabascOSV", DEBRIS, 13, 2, 4);
DFRTank tank;
//PololuQik2s9v1 qik(2, 3, 4);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

boolean finished = false;

void setup() {

  Serial.begin(9600);
  //qik.init();

  if (tcs.begin()){
    Serial.println("Found color sensor");
  } else {
    Serial.println("ERROR: Did not find color sensor");
    while(1); 
  }
  
  while (!enes.retrieveDestination());
  while (!enes.updateLocation());

}

void loop() {

  printDetails();
  if (!finished){
    readColorSensor();
  }
  delay(100);

  //Code for moving/turning, currently disabled
  /*while (abs(enes.location.theta) > 0.05) {

    qik.setM0Speed(127);
    qik.setM1Speed(-127);

    while (!enes.updateLocation());
    
  }
 
  qik.setM0Speed(127);
  qik.setM1Speed(127);

  delay(1000);
  qik.setM0Speed(0);
  qik.setM1Speed(0);
  delay(2000); */
}

//For mission objective 
void readColorSensor(){
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
  Serial.println(material);
  
  /*Serial.print("\t"); Serial.print((String) (int) r);
  Serial.print("\t"); Serial.print((String) (int) g);
  Serial.print("\t"); Serial.print((String)(int) b);
  Serial.print("\t");
  Serial.print(material);
  Serial.println(); */ 

  if (material == "Copper"){
    enes.baseObjective(COPPER);
    finished = true;
  } else if (material == "Steel"){
    enes.baseObjective(STEEL);
    finished = true;
  } else {
    delay(1000);
    readColorSensor();
  }
}


//For RF Communications Test
void printDetails(){
  while (!enes.retrieveDestination());
  while (!enes.updateLocation());
  Serial.print("x (destination):"); Serial.print(enes.destination.x);
  Serial.print(" ");
  Serial.print("y (destination):"); Serial.print(enes.destination.y);
  Serial.print(" ");
  Serial.print("x (OSV):"); Serial.print(enes.location.x);
  Serial.print(" ");
  Serial.print("y (OSV):"); Serial.print(enes.location.y);
  Serial.print(" ");
  Serial.print("theta (OSV):"); Serial.print(enes.location.theta);
  Serial.println("");
  enes.print("theta (OSV): "); enes.print(enes.location.theta);
}

String getMaterialType(int r, int g, int b){
  String material = "Unknown";
  if (r >= 91 && r <= 94 && g >= 90 && g <= 92 && b >= 65 && b <= 67){
    material = "Air";
  } else if (r >= 91 && r <= 113 && g >= 84 && g <= 92 && b >= 56 && b <= 72){
    material = "Copper";
  } else if (r >= 60 && r <= 86 && g >= 87 && g <= 101 && b >= 70 && b <= 88){
    material = "Steel";
  } 

  return material;
}
