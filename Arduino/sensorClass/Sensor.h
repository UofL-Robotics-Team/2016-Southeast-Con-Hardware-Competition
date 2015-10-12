
#ifndef Sensor_h
#define Sensor_h

#include "Arduino.h"

enum Type{
  WALL,
  DISTANCE,
  GENERIC
};

class Sensor
{
  public:
    Sensor(Type whatKind, int pin);
    Sensor(Type whatKind, int pin, int pin2);
    int reed(Type whatKind);
    float floatyRadar();
  private:
    int _pin;
    int _pin2;
    Type _whatKind;
    
};

#endif
