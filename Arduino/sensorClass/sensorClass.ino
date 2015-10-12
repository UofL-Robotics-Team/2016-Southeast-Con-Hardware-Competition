

#include "Sensor.h"

Sensor radar(DISTANCE, 13, 10);      //(trig pin, echo pin)
Sensor limitSwitch(WALL, 12);
Sensor genericSensor(GENERIC, 9);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
                                        //examples of each function below
int doop = radar.reed(DISTANCE);        //returns sonar distance integer. For more specificty, use floatyRadar()
int derp = limitSwitch.reed(WALL);      //returns boolean value
int poop = genericSensor.reed(GENERIC); //your run-o-the-mill analogRead
int peep = radar.floatyRadar();         //sonar that returns float (more specificity)
//Serial.println(radar.reed(DISTANCE);

}
