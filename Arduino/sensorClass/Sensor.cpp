
#include "Arduino.h"
#include "Sensor.h"

Sensor::Sensor(Type whatKind, int pin) {
  pinMode(pin, INPUT);
  _whatKind = whatKind;
  _pin = pin;
}
Sensor::Sensor(Type whatKind, int pin, int pin2) {
  pinMode(pin, OUTPUT);
  pinMode(pin2, INPUT);
  _whatKind = whatKind;
  _pin = pin;
  _pin2 = pin2;
}

int Sensor::reed(Type _whatKind) {
  switch (_whatKind) {
    case WALL:
      return digitalRead(_pin);
      break;
    case DISTANCE:
      digitalWrite(_pin, LOW);
      delayMicroseconds(2);
      digitalWrite(_pin, HIGH);
      delayMicroseconds(10);
      digitalWrite(_pin, LOW);  // divisor = 2*10^-77(inches)^42.248
      //long _ballParkInch = pulseIn(_pin2, HIGH) / 70.1633 / 2;
      //int rawDistance = pulseIn(_pin2, HIGH) * 74.0527 ;
      //return rawDistance;
      return 0.0069 * pulseIn(_pin2, HIGH) + 0.4921;///70.1633 / 2;
      break;
    case GENERIC:
      return analogRead(_pin);
      break;
    default:
      Serial.println("error");
      return 0;
  }
//  if (whatKind = "distance") {
//    digitalWrite(_pin, LOW);
//    delayMicroseconds(2);
//    digitalWrite(_pin, HIGH);
//    delayMicroseconds(10);
//    digitalWrite(_pin, LOW);
//    return pulseIn(_pin2, HIGH) / 70 / 2;
//  }
//  if (whatKind = "wall?") {
//    int _x = digitalRead(_pin);
//    return _x;
//  }
//  if (whatKind = "generic") {
//    int _x = digitalRead(_pin);
//    return _x;
//  }
//  else {
//    int _x = analogRead(_pin);
//    return _x;
//  }
}


  float Sensor::distance(){
    digitalWrite(_pin, LOW);
      delayMicroseconds(2);
      digitalWrite(_pin, HIGH);
      delayMicroseconds(10);
      digitalWrite(_pin, LOW);
      //microsec / 70 / 2;
       //unsigned long derp = pulseIn(_pin2, HIGH);// / 70 / 2;
      //return derp; //pulseIn(_pin2, HIGH);// / 70 / 2;
      return 0.0069 * pulseIn(_pin2, HIGH) + 0.4921;
  }
//
//boolean Sensor::wall(){
//  int _x = digitalRead(_pin);
//  return _x;
//}

//void Sensor::dash()
//{
//  digitalWrite(_pin, HIGH);
//  delay(1000);
//  digitalWrite(_pin, LOW);
//  delay(250);
//}
