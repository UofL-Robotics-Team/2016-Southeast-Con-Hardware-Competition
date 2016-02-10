/**
 * Drive firmware for Sebastian. 
 * 
 * Values for microstepping:
 * --------------------------
 * | Frequency  | Microstep |
 * |------------------------| 
 * |    >=700   |    16     |
 * |    ?       |     8     |
 * |    >=150   |     4     |
 * |    2       |     -     |
 * |    ?       |     1     |
 * --------------------------
 * 
 * Author: William Funke
 * Date: 9/28/2015
 */

#define frontLeftP        2
#define frontRightP       4
#define backLeftP         6
#define backRightP        8
#define frontLeftDirP     3
#define frontRightDirP    5
#define backLeftDirP      7
#define backRightDirP     9

#define MS1               31  // Blue wire
#define MS2               33  // Green wire
#define MS3               35  // Orange wire

#define DEFAULTMICROSTEP  2
#define DEFAULTFREQ       0
#define MAXFREQ           500

int targetForward;
int targetRight;
int targetClockwise;

int forward;
int right;
int clockwise;

int microMultiplier;

void setInterrupt1(int);
void setInterrupt2(int);
void setInterrupt3(int);
void setInterrupt4(int);
int correctMicrostep(int);
void selectMicrostep(int);
int ramp(int, int);

char readState = '0';

void setup()
{
	pinMode(frontLeftP, OUTPUT);
  pinMode(frontRightP, OUTPUT);
  pinMode(backLeftP, OUTPUT);
  pinMode(backRightP, OUTPUT);

  pinMode(frontLeftDirP, OUTPUT);
  pinMode(frontRightDirP, OUTPUT);
  pinMode(backLeftDirP, OUTPUT);
  pinMode(backRightDirP, OUTPUT);

  pinMode(MS1,OUTPUT);
  pinMode(MS2,OUTPUT);
  pinMode(MS3,OUTPUT);

  Serial.begin(9600);
  delay(1000);

  Serial.println("Enter f<number> to set forward velocity");
  Serial.println("Enter r<number> to set right velocity");
  Serial.println("Enter c<number> to set clockwise velocity");
  Serial.println("Enter m<number> to set microstep amount");
  Serial.println("Any number of these commands may be grouped together in a line;");
  Serial.println("order does not matter.");

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12 | 1 << CS10);

  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3B |= (1 << WGM32);  
  TCCR3B |= (1 << CS32 | 1 << CS30); 

  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4B |= (1 << WGM42);
  TCCR4B |= (1 << CS42 | 1 << CS40); 

  TCCR5A = 0;
  TCCR5B = 0;
  TCCR5B |= (1 << WGM52);
  TCCR5B |= (1 << CS52 | 1 << CS50);

  selectMicrostep(correctMicrostep(DEFAULTFREQ));

  setInterrupt1(DEFAULTFREQ);
  setInterrupt2(DEFAULTFREQ);
  setInterrupt3(DEFAULTFREQ);
  setInterrupt4(DEFAULTFREQ);
  interrupts();

  targetForward = DEFAULTFREQ;
  targetRight = 0;
  targetClockwise = 0;

  forward = 0;
  right = 0;
  clockwise = 0;
}

ISR(TIMER1_COMPA_vect)
{
  digitalWrite(frontLeftP, HIGH);
  digitalWrite(frontLeftP, LOW);
}

ISR(TIMER3_COMPA_vect)
{
  digitalWrite(frontRightP, HIGH);
  digitalWrite(frontRightP, LOW);
}

ISR(TIMER4_COMPA_vect)
{
  digitalWrite(backLeftP, HIGH);
  digitalWrite(backLeftP, LOW);
}

ISR(TIMER5_COMPA_vect)
{
  digitalWrite(backRightP, HIGH);
  digitalWrite(backRightP, LOW);
}

void loop()
{

  int counter = 0;
  if(Serial.available() > 0)
  {
    do
    {
      readState = Serial.read();

      switch(readState)
      {
        case 'f':
          targetForward = Serial.parseInt();
          readState = 'o';
          break;
        case 'r':
          targetRight = Serial.parseInt();
          readState = 'o';
          break;
        case 'c':
          targetClockwise = Serial.parseInt();
          readState = 'o';
          break;
        case 'm':
          selectMicrostep(Serial.parseInt());
          readState = 'o';
          break;
        default:
          readState = 'i';
      }
    }
    while(readState != 'i');
  }

  if(forward != targetForward || right != targetRight || clockwise != targetClockwise)
  {
    forward += ramp(forward,targetForward);
    right += ramp(right, targetRight);
    clockwise += ramp(clockwise,targetClockwise);
    
    int frontLeft = forward + clockwise + right;
    int frontRight = forward - clockwise - right;
    int backLeft = forward + clockwise - right;
    int backRight = forward - clockwise + right;

    int max = abs(frontLeft);
    if (abs(frontRight) > max) max = abs(frontRight);
    if (abs(backLeft) > max) max = abs(backLeft);
    if (abs(backRight) > max) max = abs(backRight);

    if(max > MAXFREQ)
    {
      frontLeft=frontLeft/max*MAXFREQ; 
      frontRight=frontRight/max*MAXFREQ; 
      backLeft=backLeft/max*MAXFREQ; 
      backRight=backRight/max*MAXFREQ;
    }

    selectMicrostep(correctMicrostep(max));

    setInterrupt1(frontLeft);
    setInterrupt2(frontRight);
    setInterrupt3(backLeft);
    setInterrupt4(backRight);
    delay(5);
  }
}


int ramp(int from, int target)
{
  int rampFactor = 5;
  int difference = abs(from - target);
  if(difference < rampFactor) rampFactor = difference;
  if(from > target) return -rampFactor;
  else if(from < target) return rampFactor;
  return 0;
}

void setInterrupt1(int frequency){
  TCNT1 = 0;

  OCR1A = 15625/abs(frequency*microMultiplier)-1;//// = (clockFreq) / (desiredFreq*preScalar) - 1 (must be <65536)
  TIMSK1 |= (1 << OCIE1A);

  Serial.println(frequency);
  if(frequency == 0) TIMSK1 &= ~(1 << OCIE1A);
  else if(frequency > 0) digitalWrite(frontLeftDirP, HIGH);
  else if(frequency < 0) digitalWrite(frontLeftDirP, LOW);
}

void setInterrupt2(int frequency)
{
  TCNT3 = 0;

  OCR3A = 15625/abs(frequency*microMultiplier)-1;
  TIMSK3 |= (1 << OCIE3A);

  if(frequency == 0) TIMSK3 &= ~(1 << OCIE3A);
  else if(frequency > 0) digitalWrite(frontRightDirP, LOW);
  else if(frequency < 0) digitalWrite(frontRightDirP, HIGH);
}

void setInterrupt3(int frequency)
{
  TCNT4 = 0;

  OCR4A = 15625/abs(frequency*microMultiplier)-1;
  TIMSK4 |= (1 << OCIE4A);

  if(frequency == 0) TIMSK4 &= ~(1 << OCIE4A);
  else if(frequency > 0) digitalWrite(backLeftDirP, HIGH);
  else if(frequency < 0) digitalWrite(backLeftDirP, LOW);
}

void setInterrupt4(int frequency){
  TCNT5 = 0;

  OCR5A = 15625 / abs(frequency * microMultiplier) - 1;
  TIMSK5 |= (1 << OCIE5A); 

  if(frequency == 0) TIMSK5 &= ~(1 << OCIE5A);
  else if(frequency > 0) digitalWrite(backRightDirP, LOW);
  else if(frequency < 0) digitalWrite(backRightDirP, HIGH);
}

int correctMicrostep(int velo) 
{
  if(velo <= 150) return 16;
  else if(velo <= 350) return 8;
  else if(velo <= 500) return 4;
  else if(velo <=650) return 2;
  else return 1;
}

void selectMicrostep(int amount)
{
  switch(amount)
  {
    case 1:
      digitalWrite(MS1,0);
      digitalWrite(MS2,0);
      digitalWrite(MS3,0);
      microMultiplier = amount;
      break;
    case 2:
      digitalWrite(MS1,1);
      digitalWrite(MS2,0);
      digitalWrite(MS3,0);
      microMultiplier = amount;
      break;
    case 4:
      digitalWrite(MS1,0);
      digitalWrite(MS2,1);
      digitalWrite(MS3,0);
      microMultiplier = amount;
      break;
    case 8:
      digitalWrite(MS1,1);
      digitalWrite(MS2,1);
      digitalWrite(MS3,0);
      microMultiplier = amount;
      break;
    case 16:
      digitalWrite(MS1,1);
      digitalWrite(MS2,1);
      digitalWrite(MS3,1);
      microMultiplier = amount;
      break;
  }
}
