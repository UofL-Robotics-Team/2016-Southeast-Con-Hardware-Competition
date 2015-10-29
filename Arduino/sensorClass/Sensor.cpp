
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
}
  float Sensor::floatyRadar(){
    noInterrupts();
	const int _numReadings = 3;
	float qVal = 1;
	float readings[_numReadings];
	float sum = 0;
	float range;
	float qValueLow;
	float qValueHigh;

	//Table of Q values
	switch (_numReadings) {
	case 3: qVal = 0.99; break;
	case 4: qVal = 0.93; break;
	case 5: qVal = 0.82; break;
	case 6: qVal = 0.74; break;
	case 7: qVal = 0.68; break;
	case 8: qVal = 0.63; break;
	}

	//populate array with sonar readings
	for (int i = 0; i < _numReadings; i++) {
		digitalWrite(_pin, LOW);
		delayMicroseconds(2);
		digitalWrite(_pin, HIGH);
		delayMicroseconds(10);
		digitalWrite(_pin, LOW);
		readings[i] = 0.0069 * pulseIn(_pin2, HIGH) + 0.4921;
		sum = sum + readings[i];
	}
	//sort array from least to greatest
	for (int i = 0; i < (_numReadings - 1); i++) {
		for (int o = 0; o < (_numReadings - (i + 1)); o++) {
			if (readings[o] > readings[o + 1]) {
				float t = readings[o];
				readings[o] = readings[o + 1];
				readings[o + 1] = t;
			}
		}
	}
	//determine Q values for low and high ends of range
	range = abs(readings[0] - readings[(_numReadings - 1)]);
	qValueLow = abs((readings[0] - readings[1]) / range);
	qValueHigh = abs((readings[(_numReadings-1)] - readings[_numReadings-2]) / range);
	//if the low end value deviates, don't include it in sum
	if (qValueLow > qVal) {
		sum = 0;
		for (int i = 1; i <= _numReadings; i++) {
			sum = sum + readings[i];
		}
		return sum / (_numReadings - 1);
	}
	//if the high end deviates,don't include it in sum
	else if (qValueHigh > qVal) {
		sum = 0;
		for (int i = 0; i < _numReadings; i++) {
			sum = sum + readings[i];
		}
		return sum / (_numReadings - 1);
	}
	//if neither end deviates, return the avg like normal
	else { return sum / _numReadings; }
      interrupts();
  }
  

