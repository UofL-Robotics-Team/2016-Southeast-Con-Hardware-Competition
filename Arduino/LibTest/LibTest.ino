#include "Drivetrain.h"


#define frontLeftP	 2
#define frontRightP	4
#define backLeftP		6
#define backRightP	 8

#define frontLeftDirP	 3
#define frontRightDirP	5
#define backLeftDirP		7
#define backRightDirP	 9

#define MS1 31	//Blue wire
#define MS2 33	//Green wire
#define MS3 35	//Orange wire


int control(double ,double ,int ,int , ControlType type);

Drivetrain drive(MS1, MS2, MS3);


const int chA=10;	//Constant variables relating to pin locations
const int chB=11;
const int chC=12;

int ch1;	//Varibles to store and display the values of each channel
int ch2;
int ch3;

int forward_speed;
int right_speed;
int clockwise_speed;

double target_x;
double current_x;

int set_angle;
int current_angle;

void readSerial();
void readSerial1();
void readReceiver();

void setup(){
	noInterrupts();


	pinMode(chA, INPUT);
	pinMode(chB,INPUT);
	pinMode(chC,INPUT);

	Serial.begin(9600);
	Serial1.begin(9600);
	delay(1000);	

	drive.attachStepper(FRLEFT, frontLeftP, frontLeftDirP, -1);
	drive.attachStepper(FRRIGHT, frontRightP, frontRightDirP, 1);
	drive.attachStepper(BKLEFT, backLeftP, backLeftDirP, -1);
	drive.attachStepper(BKRIGHT, backRightP, backRightDirP, 1);

	interrupts();
}

ISR(TIMER1_COMPA_vect){
	drive.step(FRLEFT);
}

ISR(TIMER3_COMPA_vect){
	drive.step(FRRIGHT);
}

ISR(TIMER4_COMPA_vect){
	drive.step(BKLEFT);
}

ISR(TIMER5_COMPA_vect){
	drive.step(BKRIGHT);
}
void loop(){
	readSerial();
	readSerial1();

	//Serial.println(current_x);

	//right_speed = control(target_x, current_x, -50, 0 , PROP);

	drive.setForward(forward_speed);
	drive.setRight(right_speed);
	drive.setClockwise(clockwise_speed);
	drive.updateSpeeds();
}

int control(double target,double current,int scalar,int threshold, ControlType type){
	switch(type){
		case BANG:
			if (abs(target - current) > threshold) {
				if (target - current > 0) return scalar;
				if (target - current < 0) return -scalar;
			}
			else return 0;
			break;
		case PROP:
			return scalar*(target - current);
	}
	return 0;
}

void readSerial(){
	char readState;
	if(Serial.available() > 0) {
		do{
			readState = Serial.read();
			switch(readState){
			case 'f':
				forward_speed =Serial.parseInt();
				readState = 'o';
				break;
			case 'r':
				right_speed = Serial.parseInt();
				readState = 'o';
				break;
			case 'c':
				clockwise_speed = Serial.parseInt();
				readState = 'o';
				break;
			case 'x':
				target_x = Serial.parseInt();
				readState = 'o';
				break;
			default:
				readState = 'i';
			}
		}
		while(readState!='i');
	}
}

void readSerial1() {
	char readState;
	if (Serial1.available() > 0) {
		do {
			readState = Serial1.read();
			switch (readState) {
			case 'x':
				current_x = Serial1.parseFloat();
				readState = 'o';
				break;
			default:
				readState = 'i';
			}
		} while (readState != 'i');
	}
}

void readReceiver(){

	// read the input channels
	noInterrupts();
	ch1 = pulseIn (chA,HIGH);
	ch2 = pulseIn (chB,HIGH);
	ch3 = pulseIn (chC,HIGH);
	interrupts();


	if (abs((ch1 - 1500)) < 50) ch1 = 1500;
	if (abs((ch2 - 1500)) < 50) ch2 = 1500;
	if (abs((ch3 - 1500)) < 50) ch3 = 1500;

	forward_speed = ch1-1500;
	right_speed = ch2-1500;
	clockwise_speed = ch3-1500;
}

