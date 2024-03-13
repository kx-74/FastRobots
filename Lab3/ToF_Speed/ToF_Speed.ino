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

int time_prev;
int time_curr;

void setup(void)
{
  Wire.begin();

  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");

  // Change the address of one of the ToFs for them to function simultaneously
  pinMode(XSHUT, OUTPUT);
  digitalWrite(XSHUT, LOW);             // Put sensor0 under hardware standby mode
  distanceSensor1.setI2CAddress(0x30);  // Change the address of the other sensor
  digitalWrite(XSHUT, HIGH);

  if (distanceSensor0.begin() != 0) { //Begin returns 0 on a good init
    Serial.println("Sensor 0 failed to begin. Please check wiring. Freezing...");
    while (1);
  }
  if (distanceSensor1.begin() != 0) {
    Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1);
  }

  Serial.println("Sensors online!");

  distanceSensor0.setDistanceModeShort();
  distanceSensor1.setDistanceModeLong();

  distanceSensor0.startRanging(); //Write configuration bytes to initiate measurement
  distanceSensor1.startRanging(); //Write configuration bytes to initiate measurement

  time_prev = (int) millis();
}

void loop(void) {
  if (distanceSensor0.checkForDataReady()) {
    int distance0 = distanceSensor0.getDistance();
    Serial.print("Dist_s(mm): ");
    Serial.print(distance0);
  }
  if (distanceSensor1.checkForDataReady()) {
    int distance1 = distanceSensor1.getDistance();
    Serial.print("\tDist_l(mm): ");
    Serial.print(distance1);
  }

  Serial.print("\tloop time(ms): ");
  time_curr = (int) millis();
  Serial.println(time_curr-time_prev);
  time_prev = time_curr;
}
