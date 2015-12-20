#include "Drivetrain.h"
#include "Arduino.h"


#define DEFAULTMAX 500
#define DEFAULTRAMP 10

#define X16THRESH 150
#define X8THRESH 350
#define X4THRESH 450
#define X2THRESH 600

Drivetrain::Drivetrain(int MS1, int MS2, int MS3) {
	forward.target = 0;
	right.target = 0;
	clockwise.target = 0;

	forward.current = 0;
	right.current = 0;
	clockwise.current = 0;

	this->MS1 = MS1;
	this->MS2 = MS2;
	this->MS3 = MS3;

	pinMode(MS1, OUTPUT);
	pinMode(MS2, OUTPUT);
	pinMode(MS3, OUTPUT);

	max_freq = DEFAULTMAX;
	setMSThreshold(16, X16THRESH);
	setMSThreshold(8, X8THRESH);
	setMSThreshold(4, X4THRESH);
	setMSThreshold(2, X2THRESH);

	fr_left.dir = -1;
	fr_right.dir = 1;
	bk_left.dir = -1;
	bk_right.dir = 1;

	ramp_factor = DEFAULTRAMP;
}
//Set robot relative speeds
void Drivetrain::setForward(int forward){ this->forward.target = forward;}
void Drivetrain::setRight(int right){ this->right.target = right;}
void Drivetrain::setClockwise(int clockwise){ this->clockwise.target = clockwise;}

//Set configurations
void Drivetrain::setRampFactor(int ramp){ ramp_factor = ramp;}
void Drivetrain::setMaxFreq(int max){ max_freq = max;}
void Drivetrain::setMSThreshold(int target, int frequency){
	switch(target){
	case 16:
		thresholds[0]=frequency;
		break;
	case 8:
		thresholds[1]=frequency;
		break;
	case 4:
		thresholds[2]=frequency;
		break;
	case 2:
		thresholds[3]=frequency;
		break;
	}
}

void Drivetrain::attachStepper(StepperPos stepper, int stepPin, int dirPin, int dir) {
	EasyStepper targetStepper;
	pinMode(stepPin,		OUTPUT);
	pinMode(dirPin,		 OUTPUT);
	targetStepper.step_pin = stepPin;
	targetStepper.dir_pin = dirPin;
	targetStepper.dir = dir;
	targetStepper.frequency = 0;
	switch (stepper) {
		case FRLEFT:
			fr_left = targetStepper;
			setTimer1(fr_left.frequency);
		case FRRIGHT:
			fr_right = targetStepper;
			setTimer3(fr_right.frequency);
		case BKLEFT:
			bk_left = targetStepper;
			setTimer4(bk_left.frequency);
		case BKRIGHT:
			bk_right = targetStepper;
			setTimer5(bk_right.frequency);
	}	
	setupTimers(stepper);
}

void Drivetrain::updateSpeeds() {
	if (forward.target != forward.current || right.target != right.current || clockwise.target != clockwise.current) {
		forward.current += ramp(forward);
		right.current += ramp(right);
		clockwise.current += ramp(clockwise);

		//Calculate Individual Motor Kinematics
		fr_left.frequency = forward.current + clockwise.current + right.current;
		fr_right.frequency = forward.current - clockwise.current - right.current;
		bk_left.frequency = forward.current + clockwise.current - right.current;
		bk_right.frequency = forward.current - clockwise.current + right.current;

		//Scale speeds to safe levels
		int max = abs(fr_left.frequency);
		if (abs(fr_right.frequency) > max) max = abs(fr_right.frequency);
		if (abs(bk_left.frequency) > max) max = abs(bk_left.frequency);
		if (abs(bk_right.frequency) > max) max = abs(bk_right.frequency);
		if (max > max_freq)
		{
			fr_left.frequency = fr_left.frequency / max * max_freq;
			fr_right.frequency = fr_right.frequency / max * max_freq;
			bk_left.frequency = bk_left.frequency / max * max_freq;
			bk_right.frequency = bk_right.frequency / max * max_freq;
		}
		selectMicrostep(max);

		setTimer1(fr_left.frequency);
                setDir(fr_left);
                
		setTimer3(fr_right.frequency);
                setDir(fr_right);

		setTimer4(bk_left.frequency);
                setDir(bk_left);

		setTimer5(bk_right.frequency);
                setDir(bk_right);
	}
	return;
}

void Drivetrain::step(StepperPos stepper) {
	EasyStepper targetStepper;
	switch (stepper) {
		case FRLEFT:
			targetStepper = fr_left;
			break;
		case FRRIGHT:
			targetStepper = fr_right ;
			break;
		case BKLEFT:
			targetStepper = bk_left;
			break;
		case BKRIGHT:
			targetStepper = bk_right;
			break;
	}
	digitalWrite(targetStepper.step_pin, HIGH);
	digitalWrite(targetStepper.step_pin, LOW);
}

void Drivetrain::setupTimers(StepperPos target) {
	switch (target) {
		case FRLEFT:
			TCCR1A = 0;
			TCCR1B = 0;
			TCCR1B |= (1 << WGM12);
			TCCR1B |= (1 << CS12 | 1 << CS10);
			break;
		case FRRIGHT:
			TCCR3A = 0;
			TCCR3B = 0;
			TCCR3B |= (1 << WGM32);
			TCCR3B |= (1 << CS32 | 1 << CS30);
			break;
		case BKLEFT:
			TCCR4A = 0;
			TCCR4B = 0;
			TCCR4B |= (1 << WGM42);
			TCCR4B |= (1 << CS42 | 1 << CS40);
			break;
		case BKRIGHT:
			TCCR5A = 0;
			TCCR5B = 0;
			TCCR5B |= (1 << WGM52);
			TCCR5B |= (1 << CS52 | 1 << CS50);
			break;
	}
}

int Drivetrain::ramp(Speed speed) {
	int scaledRamp = ramp_factor;
	int difference = abs(speed.current - speed.target);
	if (difference < ramp_factor) scaledRamp = difference;
	if (speed.current > speed.target) return -scaledRamp;
	else if (speed.current < speed.target) return scaledRamp;
	return 0;
}
void Drivetrain::selectMicrostep(int speed) {
	if (speed <= thresholds[0]) microstep = 16;
	else if (speed <= thresholds[1]) microstep = 8;
	else if (speed <= thresholds[2]) microstep = 4;
	else if (speed <= thresholds[3]) microstep = 2;
	else microstep = 1;
	switch (microstep) {
		case 1:
			digitalWrite(MS1, 0);
			digitalWrite(MS2, 0);
			digitalWrite(MS3, 0);
			break;
		case 2:
			digitalWrite(MS1, 1);
			digitalWrite(MS2, 0);
			digitalWrite(MS3, 0);
			break;
		case 4:
			digitalWrite(MS1, 0);
			digitalWrite(MS2, 1);
			digitalWrite(MS3, 0);
			break;
		case 8:
			digitalWrite(MS1, 1);
			digitalWrite(MS2, 1);
			digitalWrite(MS3, 0);
			break;
		case 16:
			digitalWrite(MS1, 1);
			digitalWrite(MS2, 1);
			digitalWrite(MS3, 1);
			break;
	}
}

void Drivetrain::setTimer1(int frequency) {
	TCNT1 = 0;

	OCR1A = 15625 / abs(frequency * microstep) - 1; //// = (clockFreq) / (desiredFreq*preScalar) - 1 (must be <65536)
	TIMSK1 |= (1 << OCIE1A);

	if (frequency == 0) TIMSK1 &= ~(1 << OCIE1A);						 //Disable Timer
	return;
}

void Drivetrain::setTimer3(int frequency) {
	TCNT3 = 0;

	OCR3A = 15625 / abs(frequency * microstep) - 1;
	TIMSK3 |= (1 << OCIE3A);

	if(frequency == 0) TIMSK3 &= ~(1 << OCIE3A);
	return;
}

void Drivetrain::setTimer4(int frequency) {
	TCNT4 = 0;

	OCR4A = 15625 / abs(frequency * microstep) - 1;
	TIMSK4 |= (1 << OCIE4A);

	if (frequency == 0) TIMSK4 &= ~(1 << OCIE4A);
	return;
}

void Drivetrain::setTimer5(int frequency) {
	TCNT5 = 0;

	OCR5A = 15625 / abs(frequency * microstep) - 1;
	TIMSK5 |= (1 << OCIE5A);

	if (frequency == 0) TIMSK5 &= ~(1 << OCIE5A);
	return;
}

void Drivetrain::setDir(EasyStepper stepper) {
	stepper.frequency *= stepper.dir;

	if (stepper.frequency > 0) digitalWrite(stepper.dir_pin, HIGH);
        else if(stepper.frequency < 0) digitalWrite(stepper.dir_pin, LOW);
	return;
}


