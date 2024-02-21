/*
  Reading distance from the laser based VL53L1X
  By: Nathan Seidle
  Revised by: Andy England
  SparkFun Electronics
  Date: April 4th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  SparkFun labored with love to create this code. Feel like supporting open source hardware? 
  Buy a board from SparkFun! https://www.sparkfun.com/products/14667

  This example prints the distance to an object.

  Are you getting weird readings? Be sure the vacuum tape has been removed from the sensor.
*/

#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

//Optional interrupt and shutdown pins.
#define XSHUT A0
#define INTERRUPT_PIN -1

SFEVL53L1X distanceSensor0;
SFEVL53L1X distanceSensor1;

void setup(void)
{
  Wire.begin();

  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");

  pinMode(XSHUT, OUTPUT);
  digitalWrite(XSHUT, LOW);   // Put sensor0 under standby mode
  distanceSensor1.setI2CAddress(0x30);
  digitalWrite(XSHUT, HIGH);

  if (distanceSensor0.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor 0 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  if (distanceSensor1.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }

  Serial.println("Sensors online!");

  distanceSensor0.setDistanceModeShort();
  distanceSensor1.setDistanceModeLong();
}

void loop(void)
{
  distanceSensor0.startRanging(); //Write configuration bytes to initiate measurement
  distanceSensor1.startRanging(); //Write configuration bytes to initiate measurement
  
  int distance0 = distanceSensor0.getDistance();
  distanceSensor0.clearInterrupt();
  distanceSensor0.stopRanging();

  Serial.print("Dist_s(mm): ");
  Serial.print(distance0);

  int distance1 = distanceSensor1.getDistance();
  distanceSensor1.clearInterrupt();
  distanceSensor1.stopRanging();
  
  Serial.print("\tDist_l(mm): ");
  Serial.print(distance1);

  Serial.println();
}
