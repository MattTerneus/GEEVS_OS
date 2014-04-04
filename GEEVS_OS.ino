#include <TimerOne.h>
#include <Ping.h>
#include <SPI.h>
//System Values
#define CYCLES_PER_SECOND 4
#define ROUTINES_PER_CYCLE 4
#define INTERRUPT_PERIOD 1000000/(CYCLES_PER_SECOND*ROUTINES_PER_CYCLE)
//Pin data
#define SS_PIN 24

//SPI Values
#define SPI_WRITE 0x00
#define SPI_READ 0x80
#define SPI_MULTIBYTE 0x40
#define SPI_IDLE 0xFF

//Accelerometer Values
#define ACCEL_OFFSET 0x1E
#define ACCEL_DATA 0x32

//Gyro data
#define GYRO_OFFESET 512
#define GYRO_CYCLE 516 * CYCLES_PER_SECOND

void (*routines[ROUTINES_PER_CYCLE])();
int routineIndex = 0;
volatile int headingRaw, headingDeg = 0;
volatile long xPos, yPos, xVel, yVel = 0;
volatile unsigned char pingLock = 0;
Ping pingLeft = Ping(0);
Ping pingCenter = Ping(1);
Ping pingRight = Ping(2);


void accelerometer_service ()
{
  int xDelta, yDelta;
  digitalWrite(SS_PIN, 0); //Begin Trasmission
  SPI.transfer(SPI_READ&SPI_MULTIBYTE&ACCEL_DATA); //Send Address
  xDelta = SPI.transfer(SPI_IDLE); //Read X
  xDelta |= SPI.transfer(SPI_IDLE) << 8;
  yDelta = SPI.transfer(SPI_IDLE); //Read y
  yDelta |= SPI.transfer(SPI_IDLE) << 8;
  digitalWrite(SS_PIN, 1); //End transmission
  
  //Update velocity and position
  xVel += xDelta;
  yVel += yDelta;
  xPos += xVel;
  yPos += yVel;    
}

void gyro_service ()
{
  int delta;
  delta = analogRead(0); //Read 10 bit ADC 
  headingRaw += delta-GYRO_OFFESET; //Adjust heading
  if (headingRaw >= GYRO_CYCLE) //Wrap Overflow
    headingRaw -= GYRO_CYCLE;
  if (headingRaw < 0) //Wrap Underflow
    headingRaw += GYRO_CYCLE;
    
  headingDeg = ((long)headingRaw * 360) / GYRO_CYCLE; //Update heading in degrees
}

void ping_service ()
{
  if (!pingLock)
  {
    pingCenter.fire();
  }  
}

void motor_service ()
{
  
}

void realtime_service_call ()
{
  if (routines[routineIndex] != 0)
    routines[routineIndex]();
  routineIndex++;
  if (routineIndex == ROUTINES_PER_CYCLE)
    routineIndex = 0;
}

void setup ()
{
  signed int xDelta, yDelta;
  //Configure SPI communication
  pinMode (SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, 1);
  SPI.setDataMode(3);
  SPI.begin();
  
  //Zero Accelerometer
  digitalWrite(SS_PIN, 0); //Begin Trasmission
  SPI.transfer(SPI_READ&SPI_MULTIBYTE&ACCEL_DATA); //Send Address
  xDelta = SPI.transfer(SPI_IDLE); //Read X
  xDelta |= SPI.transfer(SPI_IDLE) << 8;
  yDelta = SPI.transfer(SPI_IDLE); //Read y
  yDelta |= SPI.transfer(SPI_IDLE) << 8;
  digitalWrite(SS_PIN, 1); //End transmission
  delay(1);
  digitalWrite(SS_PIN, 0); //Begin Trasmission
  SPI.transfer(SPI_WRITE&SPI_MULTIBYTE&ACCEL_OFFSET); //Send Address
  SPI.transfer((signed char)-xDelta); //Write x offset
  SPI.transfer((signed char)-yDelta); //Write y offset
  digitalWrite(SS_PIN, 1); //End transmission
  
  //Setup real time services
  for (int i = 0; i < ROUTINES_PER_CYCLE; i++)
    routines[i] = 0;
    
  routines[0] = &accelerometer_service;
  routines[1] = &gyro_service;
  routines[2] = &ping_service;
  routines[3] = &motor_service;
  
  //Begin real time clock
  Timer1.attachInterrupt(realtime_service_call);
  Timer1.initialize(INTERRUPT_PERIOD);
}

void loop()
{
  
}

