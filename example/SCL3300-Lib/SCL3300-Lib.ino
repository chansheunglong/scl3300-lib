#include "SCL3300.h"
Murata_SCL3300 SCL3300;

float x, y, z;

void setup() {
  Serial.begin(9600);
  
  //Config the Chip Select Pin
  SCL3300.setCSPin(9);

  //Wake up the sensor
  SCL3300.wake();

  //Sensor need 1s start-up time
  delay(1000);
}

void loop() {
  //Read X, Y, Z angle
  SCL3300.angle(x, y, z);

  //Print the result to serial monitor
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.println(z);

  //Wait
  delay(500);
}
