
#ifndef Sensor_h
#define Sensor_h

#include "Arduino.h"

//#include "Wire.h"
////#include <Adafruit_Sensor.h>
//#include "Adafruit_BNO055.h"
////#include <utility/imumaths.h>


enum Type{
  WALL,
  DISTANCE,
  GENERIC
};

//    //int poopy = 5;
//    int _beginStopWatch;
//    int _stopStopWatch;
//    int _time;

class Sensor
{
  public:
    Sensor(Type whatKind, int pin);
    Sensor(Type whatKind, int pin, int pin2);
    int reed(Type whatKind);
    float floatyRadar();
  private:
	//int _numReadings = 3;
    int _pin;
    int _pin2;
    Type _whatKind;
    
};

#endif
