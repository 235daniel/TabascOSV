#include <Wire.h>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

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
