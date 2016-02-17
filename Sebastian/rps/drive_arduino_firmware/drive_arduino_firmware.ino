/**
 * Drive firmware for Sebastian. 
 *
 * Author: William Funke
 * Subauthor: Alex Bennett
 */

#include "Drivetrain.h"

#define DEBUG_ENABLED // Debug flag (COMMENT THIS OUT UNLESS A HUMAN IS OPERATING THE CONTROLS)

#define frontLeftP        9
#define frontRightP       7
#define backLeftP         5
#define backRightP        3

#define frontLeftDirP     8
#define frontRightDirP    6
#define backLeftDirP      4
#define backRightDirP     2

#define ENABLE            52
#define MS1               50
#define MS2               48 
#define MS3               46

// Transmitter channels
#define CH1               10
#define CH2               11
#define CH3               12

// Transmitter data
int ch1 = 0, ch2 = 0, ch3 = 0;

// Limit switch pins
#define LIMIT_SWT_NORTH   47
#define LIMIT_SWT_SOUTH   49
#define LIMIT_SWT_EAST    51
#define LIMIT_SWT_WEST    53

// Create drivetrain
Drivetrain drive(MS1, MS2, MS3);

// Define drivetrain reference variables
int forward_speed;
int right_speed;
int clockwise_speed;

double target_x;
double current_x;

int set_angle;
int current_angle;

ISR(TIMER1_COMPA_vect)
{
  drive.step(FRLEFT);
}

ISR(TIMER3_COMPA_vect)
{
  drive.step(FRRIGHT);
}

ISR(TIMER4_COMPA_vect)
{
  drive.step(BKLEFT);
}

ISR(TIMER5_COMPA_vect)
{
  drive.step(BKRIGHT);
}

void setup()
{
  // Temporarily disable interrupts
  noInterrupts();

  // Setup limit switches
  pinMode(LIMIT_SWT_NORTH, INPUT);
  pinMode(LIMIT_SWT_SOUTH, INPUT);
  pinMode(LIMIT_SWT_EAST, INPUT);
  pinMode(LIMIT_SWT_WEST, INPUT);

  // Transmitter inputs
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);

  // Start serial and print directions
  Serial.begin(115200);
  delay(1000);

  // Print debug information
  printDebug("Enter f<number> to set forward velocity");
  printDebug("Enter r<number> to set right velocity");
  printDebug("Enter c<number> to set clockwise velocity");
  printDebug("Enter z<number 1-4> to zero in the specified direction");
  printDebug("Any number of these commands may be grouped together in a line; order does not matter.");

  drive.attachStepper(FRLEFT, frontLeftP, frontLeftDirP, -1);
  drive.attachStepper(FRRIGHT, frontRightP, frontRightDirP, 1);
  drive.attachStepper(BKLEFT, backLeftP, backLeftDirP, -1);
  drive.attachStepper(BKRIGHT, backRightP, backRightDirP, 1);

  // Re-enable interrupts
  interrupts();

  // Enable stepper drivers
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, LOW);
}

void loop()
{
  // Grab serial input
  readSerial();
  //readReceiver();

  // Update drivetrain
  drive.setForward(forward_speed);
  drive.setRight(right_speed);
  drive.setClockwise(clockwise_speed);
  drive.updateSpeeds();
}

void zeroDirection(int direction) // 1 = NORTH, 2 = SOUTH, 3 = EAST, 4 = WEST
{
  switch(direction)
  {
    // Zero north
    case 1: 
      // Drive until switch is hit
      while(!digitalRead(LIMIT_SWT_NORTH))
      {
         // Set target velocities
        drive.setForward(100);
        drive.setRight(0);
        drive.setClockwise(0);
        drive.updateSpeeds();
      }

      // Switch was hit, set speed to 0
      drive.setForward(0);
      drive.setRight(0);
      drive.setClockwise(0);
      drive.updateSpeeds();

      // Break case
      break;

    // Zero south
    case 2:
      // Drive until switch is hit
      while(!digitalRead(LIMIT_SWT_SOUTH))
      {
         // Set target velocities
        drive.setForward(-100);
        drive.setRight(0);
        drive.setClockwise(0);
        drive.updateSpeeds();
      }

      // Switch was hit, set speed to 0
      drive.setForward(0);
      drive.setRight(0);
      drive.setClockwise(0);
      drive.updateSpeeds();

      // Break case
      break;

    // Zero east
    case 3:
      // Drive until switch is hit
      while(!digitalRead(LIMIT_SWT_EAST))
      {
         // Set target velocities
        drive.setForward(0);
        drive.setRight(100);
        drive.setClockwise(0);
        drive.updateSpeeds();
      }

      // Switch was hit, set speed to 0
      drive.setForward(0);
      drive.setRight(0);
      drive.setClockwise(0);
      drive.updateSpeeds();

      // Break case
      break;

    // Zero west
    case 4:
      // Drive until switch is hit
      while(!digitalRead(LIMIT_SWT_WEST))
      {
         // Set target velocities
        drive.setForward(0);
        drive.setRight(-100);
        drive.setClockwise(0);
        drive.updateSpeeds();
      }

      // Switch was hit, set speed to 0
      drive.setForward(0);
      drive.setRight(0);
      drive.setClockwise(0);
      drive.updateSpeeds();

      // Break case
      break;

    default:
      break;
  }
}

void readSerial()
{
  char readState;

  if(Serial.available() > 0) 
  {
    do
    {
      readState = Serial.read();

      switch(readState)
      {
        case 'f':
          forward_speed = Serial.parseInt();
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
        case 'z':
          zeroDirection(Serial.parseInt());
          readState = 'o';
          break;
        default:
          readState = 'i';
      }
    }
    while(readState != 'i');
  }
}

void readReceiver()
{
  // read the input channels
  noInterrupts();
  ch1 = pulseIn(CH1, HIGH);
  ch2 = pulseIn(CH2, HIGH);
  ch3 = pulseIn(CH3, HIGH);
  interrupts();


  if (abs((ch1 - 1500)) < 50) ch1 = 1500;
  if (abs((ch2 - 1500)) < 50) ch2 = 1500;
  if (abs((ch3 - 1500)) < 50) ch3 = 1500;

  forward_speed = ch1-1500;
  right_speed = ch2-1500;
  clockwise_speed = ch3-1500;

  Serial.println("F: " + String(forward_speed) + ", R: " + String(right_speed) + ", CW: " + String(clockwise_speed));
}

void printDebug(String msg)
{
  #ifdef DEBUG_ENABLED
    Serial.println(msg);
  #endif
}
