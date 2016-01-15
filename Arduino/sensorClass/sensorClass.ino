

#include "Sensor.h"



Sensor radar(DISTANCE, 13, 10);      //(trig pin, echo pin)
Sensor limitSwitch(WALL, 12);
Sensor genericSensor(GENERIC, 9);
int time = 0;
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

//testing interrupt protection
time = millis() - time;
if(time/2000 >= 1){
int peep = radar.floatyRadar();         //sonar that returns float (more precision)
}
Serial.println(radar.reed(DISTANCE));

}

