#include <Wire.h>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
double[] copper = {107.5,85.8,61.3};
double[] steel = {82.1,94.9,74.8};
double copperStDev = 4.5, steelStDev = 5.7; 

void setup() {
  Serial.begin(9600);
  if (tcs.begin()){
    Serial.println("Found sensor");
  } else {
    Serial.println("Error: Did not find sensor");
       while(1); 
  }
}



void loop() {
  uint16_t clear, red, green, blue;
  tcs.setInterrupt(false); // Turn on LED
  delay(60);
  tcs.getRawData(&red, &green, &blue, &clear);
  
  tcs.setInterrupt(true); // Turn off LED

  Serial.print("C:\t"); Serial.print(clear);
  Serial.print("\tR:\t"); Serial.print(red);
  Serial.print("\tG:\t"); Serial.print(green);
  Serial.print("\tB:\t"); Serial.print(blue);

  uint32_t sum = clear;
  float r, g, b;
  
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;

  Serial.print("\t");
  Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);

  //Testing version of determining the material detected
  Serial.print("\t");

  String material = getMaterialType((int)r, (int)g, (int)b);
  Serial.print("\t"); Serial.print((String) (int) r);
  Serial.print("\t"); Serial.print((String) (int) g);
  Serial.print("\t"); Serial.print((String)(int) b);
  Serial.print("\t");
  Serial.print(material);
  Serial.println();
}
String getMaterialType(int r, int g, int b){
  double copperDist = sqrt(pow(copper[0] - r, 2) + pow(copper[1] - g, 2) + pow(copper[2] - b, 2));
  double steelDist = sqrt(pow(steel[0] - r, 2) + pow(steel[1] - g, 2) + pow(steel[2] - b, 2));

  if(copperDist < steelDist){
    if(copperDist < 3*copperStDev){
      return "Copper";
    }
    return "No Material";
  }

  if(steelDist < 3*steelStDev){
    return "Steel";
  }

  return "No Material";
}
