#include "I2Cdev.h"
#include "digitalWriteFast.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif
#include <SPI.h>
#include <avr/pgmspace.h>
#include "adns9800_firmware.h"

// --- GENERAL -----------------------------------------------------
#define LED_PIN 13
bool blinkState = false;

// --- ADNS 9800 ---------------------------------------------------
byte initComplete = 0;
byte testctr = 0;
unsigned long currTime;
unsigned long timer;
volatile int32_t xdata;
volatile int32_t ydata;
//volatile byte xydata[4];
//int16_t* x = (int16_t*) &xydata[0];
//int16_t* y = (int16_t*) &xydata[2];
volatile byte movementflag = 0;
extern const unsigned short firmware_length;
extern const char firmware_data[];
int64_t xdistance = 0;
int64_t ydistance = 0;
uint32_t laserPollTimer = 0;
uint32_t lastTime = 0;
const int NCS_PIN = 10;

// --- MPU 6050 ----------------------------------------------------
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Interrupt detection routine
volatile bool mpuInterrupt = false;

void dmpDataReady()
{
  mpuInterrupt = true;
}

void adns_com_begin()
{
  digitalWriteFast(NCS_PIN, LOW);
}

void adns_com_end()
{
  digitalWriteFast(NCS_PIN, HIGH);
}

byte adns_read_reg(byte reg_addr)
{
  adns_com_begin();

  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(100); // tSRAD
  // read data
  byte data = SPI.transfer(0);

  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
  adns_com_end();
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-NCS
  return data;
}

void adns_write_reg(byte reg_addr, byte data)
{
  adns_com_begin();

  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);

  delayMicroseconds(20); // tSCLK-NCS for write operation
  adns_com_end();
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound
}

void adns_upload_firmware()
{
  // send the firmware to the chip, cf p.18 of the datasheet
  // set the configuration_IV register in 3k firmware mode
  adns_write_reg(REG_Configuration_IV, 0x02); // bit 1 = 1 for 3k mode, other bits are reserved

  // write 0x1d in SROM_enable reg for initializing
  adns_write_reg(REG_SROM_Enable, 0x1d);

  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low

  // write 0x18 to SROM_enable to start SROM download
  adns_write_reg(REG_SROM_Enable, 0x18);

  // write the SROM file (=firmware data)
  adns_com_begin();
  SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);

  // send all bytes of the firmware
  unsigned char c;
  for (int i = 0; i < firmware_length; i++) {
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  adns_com_end();
}

void adns_start()
{
  adns_com_end(); // ensure that the serial port is reset
  adns_com_begin(); // ensure that the serial port is reset
  adns_com_end(); // ensure that the serial port is reset
  adns_write_reg(REG_Power_Up_Reset, 0x5a); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  adns_read_reg(REG_Motion);
  adns_read_reg(REG_Delta_X_L);
  adns_read_reg(REG_Delta_X_H);
  adns_read_reg(REG_Delta_Y_L);
  adns_read_reg(REG_Delta_Y_H);
  // upload the firmware
  adns_upload_firmware();
  delay(10);
  adns_write_reg(REG_Configuration_I, 0x01); // 50 cpi
  delay(10);
  //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
  // reading the actual value of the register is important because the real
  // default value is different from what is said in the datasheet, and if you
  // change the reserved bytes (like by writing 0x00...) it would not work.
  byte laser_ctrl0 = adns_read_reg(REG_LASER_CTRL0);
  adns_write_reg(REG_LASER_CTRL0, laser_ctrl0 & 0xf0 );
  delay(1);
}

void adns_disp_registers()
{
  int oreg[7] = {
    0x00, 0x3F, 0x2A, 0x02
  };

  char* oregname[] = {
    "Product_ID", "Inverse_Product_ID", "SROM_Version", "Motion"
  };

  byte regres;

  adns_com_begin();

  printDebug("--- ADNS 9800 Registers ------------------------------");

  int rctr = 0;
  for (rctr = 0; rctr < 4; rctr++) 
  {
    SPI.transfer(oreg[rctr]);
    delay(1);
    
    #ifdef DEBUG_ENABLED
      Serial.print(String(oregname[rctr]) + ": ");
      Serial.print(oreg[rctr], HEX);
      Serial.print(" (HEX), ");
    #endif
    
    regres = SPI.transfer(0);
    
    #ifdef DEBUG_ENABLED
      Serial.print(regres, BIN);
      Serial.print(" (BIN), ");
      Serial.print(regres, HEX);
      Serial.println(" (HEX)");
    #endif
    
    delay(1);
  }

  printDebug("------------------------------------------------------");
  
  adns_com_end();
}

int convert_twos_comp(int b)
{
  // Convert from 2's complement
  if (b & 0x80)
  {
    b = -1 * ((b ^ 0xff) + 1);
  }
  return b;
}

void setup()
{
  // Begin serial
  Serial.begin(115200);

  // --- ADNS 9800 -------------------------------------------------

  // Setup NCS pin
  pinMode(NCS_PIN, OUTPUT);

  // Begin SPI for ADNS 9800
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(8);

  // Startup ADNS
  adns_start();
  adns_disp_registers();
  delay(100);
  initComplete = 9;

  // --- MPU 6050 --------------------------------------------------

  // Begin I2C/Fastwire for MPU 6050
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // Initialize the MPU
  mpu.initialize();
  if(!mpu.testConnection())
  {
    printDebug("Connection to MPU failed! Program halted.");
    return;
  }

  // Initialize and configure the DMP on the MPU
  devStatus = mpu.dmpInitialize();

  // Set GYRO offsets
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(41);
  mpu.setZAccelOffset(1788);

  // Check MPU status to ensure successful startup
  if(devStatus == 0)
  {
    // Enable DMP
    mpu.setDMPEnabled(true);

    // Enable interrupt
    attachInterrupt(digitalPinToInterrupt(3), dmpDataReady, RISING); // This is pin 2
    mpuIntStatus = mpu.getIntStatus();

    // Set DMP ready flag so loop can safely begin
    dmpReady = true;

    // Get expected DMP packet size for comparison later
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // MPU failed to startup
    printDebug("MPU failed to startup. Error code: " + String(devStatus)); // 1 = initial memory load failed, 2 = DMP configuration updates failed
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

void loop()
{
  while(dmpReady && !mpuInterrupt && fifoCount < packetSize)
  {
    // Grab ADNS data periodically, when available
    if (initComplete == 9 && (millis() - laserPollTimer) > 5 && !digitalRead(3))
    {      
      /*
      // Read motion data
      xydata[0] = (byte) adns_read_reg(REG_Delta_X_L);
      xydata[1] = (byte) adns_read_reg(REG_Delta_X_H);
      xydata[2] = (byte) adns_read_reg(REG_Delta_Y_L);
      xydata[3] = (byte) adns_read_reg(REG_Delta_Y_H);

      // Save last poll time
      laserPollTimer = millis();

      // Update distances
      xdistance += float(*x) / 200 * 2.54;
      ydistance += float(*y) / 200 * 2.54;
      */
      
      // Lock motion registers
      uint8_t mot_reg = (uint8_t) adns_read_reg(REG_Motion);

      // Read motion data
      xdata = (int32_t) adns_read_reg(REG_Delta_X_L);
      adns_read_reg(REG_Delta_X_H); // discard upper register
      ydata = (int32_t) adns_read_reg(REG_Delta_Y_L);
      adns_read_reg(REG_Delta_Y_H); // discard upper register

      // Concatenate upper and lower data registers
      //int32_t full_xdata = (xdata[1] << 16) | xdata[0];
      //int32_t full_ydata = (ydata[1] << 16) | ydata[0];

      // Record last poll time
      laserPollTimer = millis();

      // Add to total distance
      xdistance = xdistance + convert_twos_comp(xdata);
      ydistance = ydistance + convert_twos_comp(ydata);
    }

    char command = 0;
    
    if(Serial.available())
    {
      command = Serial.read();

      switch(command)
      {
        case 'z':
          // Reset the distance totals
          xdistance = 0;
          ydistance = 0;
          break;
        case 'p':
          // Return position information
          if((millis() - lastTime) > 100)
          {
            Serial.println(String((double) 0.00 - (double) ydistance / 50) + "," + String((double) 0.00 - (double) xdistance / 50) + "," + String(ypr[0] * 180 / M_PI));
            //Serial.println(String(ydistance) + "," + String(xdistance) + "," + String(ypr[0] * 180 / M_PI));
            lastTime = millis();
          }
          break;
        default:
          break;
      }

      Serial.flush();
    }
  }

  // Reset interrupt flag and grab interrupt status
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
 
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) // Check for FIFO overflow (hopefully doesn't happen)
  {                     
    // FIFO overflow occurred, reset FIFO        
    mpu.resetFIFO();
    printDebug("FIFO overflow!");
  }
  else if(mpuIntStatus & 0x02) // Otherwise check for DMP data
  {
    // Wait for data to be available  
    while(fifoCount < packetSize) fifoCount = mpu.getFIFOCount(); 

    // Grab the data from the FIFO buffer and reduce FIFO count
    mpu.getFIFOBytes(fifoBuffer, packetSize);                             
    fifoCount -= packetSize;

    // Calculate MPU data
    mpu.dmpGetQuaternion(&q, fifoBuffer);                                     
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Blink LED to indicate activity
    //blinkState = !blinkState;
    //digitalWriteFast(LED_PIN, blinkState);
  }
}

void printDebug(String msg)
{
  #ifdef DEBUG_ENABLED
    Serial.println(msg);
  #endif
}






