

#include "Sensor.h"

Sensor radar(DISTANCE, 13, 10);
Sensor limitSwitch(WALL, 12);
Sensor genericSensor(GENERIC, 9);
void setup()
{
  Serial.begin(9600);
}

void loop()
{
//  int derp = radar.reed();
//  Serial.println(derp);
//  boolean herp = limitSwitch.wall();
//  Serial.println(herp);
//  delay(3000);

//int doop = radar.distance();
//Serial.println(doop);

//int doop = radar.reed(DISTANCE);
//int derp = limitSwitch.reed(WALL);
//int poop = genericSensor.reed(GENERIC);

//Type sensor = GENERIC;

delay(1000);
//Serial.println(doop);

Serial.println(radar.distance());

}
