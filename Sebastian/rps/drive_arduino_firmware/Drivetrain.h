/**
 * Drivetrain to control 4 stepper motors utilizing mecanum wheels.
 *
 * Author: William Funke
 * Subauthor: Alex Bennett
 */

enum StepperPos
{
	FRLEFT, FRRIGHT, BKLEFT, BKRIGHT
};

enum ControlType 
{
	BANG,
	PROP
};

struct EasyStepper
{
	int step_pin;
	int dir_pin;
	int enable_pin;
	int frequency;
	int dir;
} typedef EasyStepper;

struct Speed
{
	int target;
	int current;
} typedef Speed;

class Drivetrain
{
	public:
		Drivetrain(int, int, int);
		
		void setForward(int forward);
		void setRight(int right);
		void setClockwise(int clockwise);
		
		void setRampFactor(int factor);
		void setMaxFreq(int max);
		void setMSThreshold(int targetMicrostep, int frequency);
		
		void attachStepper(StepperPos stepper, int stepPin, int dirPin, int dir);
		void updateSpeeds();
		void step(StepperPos stepper);

	private:
		EasyStepper fr_left;
		EasyStepper fr_right;
		EasyStepper bk_left;
		EasyStepper bk_right;
		
		Speed forward;
		Speed right;
		Speed clockwise;

		int max_freq;
		int ramp_factor;
		int microstep;
		int thresholds[4];

		int MS1, MS2, MS3;
		
		void setupTimers(StepperPos);
		int ramp(Speed speed);
		void selectMicrostep(int speed); 

		void setDir(EasyStepper stepper);
		
		void setTimer1(int frequency);
		void setTimer3(int frequency);
		void setTimer4(int frequency);
		void setTimer5(int frequency);
};

