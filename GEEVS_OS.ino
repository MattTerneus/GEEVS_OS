#include <TimerOne.h>
#include <Ping.h>
#include <SPI.h>
//System Values
#define CYCLES_PER_SECOND 4
#define ROUTINES_PER_CYCLE 4
#define INTERRUPT_PERIOD 1000000/(CYCLES_PER_SECOND*ROUTINES_PER_CYCLE)

//Pin data
#define SS_PIN 10
#define MOTOR_R_PIN 5
#define MOTOR_L_PIN 6

//SPI Values
#define SPI_WRITE 0x00
#define SPI_READ 0x80
#define SPI_MULTIBYTE 0x40
#define SPI_IDLE 0x00

//Accelerometer Values
#define ACCEL_OFFSET 0x1E
#define ACCEL_DATA 0x32
#define DATA_FORMAT 0x31

//Gyro data
#define GYRO_OFFESET 512
#define GYRO_CYCLE 96 * CYCLES_PER_SECOND

//Motor data
#define MOVE_TURN 1
#define MOVE_FORWARD 2
#define IDLE_SPEED 127
#define BASE_SPEED 32
#define ROTATION_DEADZONE 6
#define POSITION_DEADZONE 10

void (*routines[ROUTINES_PER_CYCLE])();
volatile signed int headingRaw = 0;
volatile unsigned int headingDeg, routineIndex = 0;
volatile signed long sidewaysPos, forwardPos, sidewaysVel, forwardVel, forwardCm, sidewaysCm = 0;
unsigned char pingLock, moveCommand = 0;
volatile unsigned int testAccel, testGyro = 0;
volatile signed int goalHeading = 0, headingOffset = 0;
signed long goalPosition = 0;

Ping pingLeft = Ping(2);
Ping pingCenter = Ping(3);
Ping pingRight = Ping(4);


void accelerometer_service ()
{
  int xDelta, yDelta;
  digitalWrite(SS_PIN, 0); //Begin Trasmission
  delay(1);
  SPI.transfer(SPI_READ&SPI_MULTIBYTE&ACCEL_DATA); //Send Address
  xDelta = SPI.transfer(SPI_IDLE); //Read X
  xDelta |= SPI.transfer(SPI_IDLE) << 8;
  yDelta = SPI.transfer(SPI_IDLE); //Read y
  yDelta |= SPI.transfer(SPI_IDLE) << 8;
  digitalWrite(SS_PIN, 1); //End transmission
  
  testAccel = xDelta;
  
  //Update velocity and position
  sidewaysVel += xDelta;
  forwardVel += yDelta;
  sidewaysPos += sidewaysVel;
  forwardPos += forwardVel;
  
  forwardCm = forwardPos * 3826 / 1000;
  sidewaysCm = sidewaysPos * 3826 / 1000;
}

void gyro_service ()
{
  int delta;
  delta = analogRead(0); //Read 10 bit ADC
  testGyro = delta;
  headingRaw += delta+headingOffset; //Adjust heading
  if (headingRaw >= GYRO_CYCLE) //Wrap Overflow
    headingRaw -= GYRO_CYCLE;
  if (headingRaw < 0) //Wrap Underflow
    headingRaw += GYRO_CYCLE;
    
  headingDeg = ((long)headingRaw * 360) / (GYRO_CYCLE); //Update heading in degrees
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
  unsigned char newSpeed = IDLE_SPEED;
  signed int headingDelta;
  
  if (moveCommand == MOVE_FORWARD) //Forward Mode
  {
    if (pingCenter.centimeters() >= 100) //Only move if there is room ahead
    {
      newSpeed += BASE_SPEED;
      if (pingCenter.centimeters() < 150) //Slow robot near obstacles
        newSpeed -= BASE_SPEED>>2;
    }
    analogWrite(MOTOR_R_PIN, newSpeed);
    analogWrite(MOTOR_L_PIN, newSpeed);
  }
  else if (moveCommand == MOVE_TURN) //Turn Mode
  {
    headingDelta = goalHeading-headingDeg;
    // > 180 degree turn compensation
    if (headingDelta > 180)
      headingDelta = 180-headingDelta;
    else if (headingDelta < -180)
      headingDelta = -180-headingDelta;
        
    if (headingDelta > ROTATION_DEADZONE)
      newSpeed += BASE_SPEED>>2;
    else if (headingDelta < -ROTATION_DEADZONE)
      newSpeed -= BASE_SPEED>>2;
         
    analogWrite(MOTOR_R_PIN, newSpeed);
    analogWrite(MOTOR_L_PIN, -newSpeed);
  }
  else //Default mode
  {
    analogWrite(MOTOR_R_PIN, IDLE_SPEED);
    analogWrite(MOTOR_L_PIN, IDLE_SPEED);
  }
}

void realtime_service_call ()
{
  if (routines[routineIndex] != 0) //Only call routine if it exists
    routines[routineIndex]();
  routineIndex++; //Advance to next routine
  if (routineIndex == ROUTINES_PER_CYCLE) //Wrap around at end of cycle
    routineIndex = 0;
}

void setup ()
{
  signed int xDelta, yDelta;
  
  Serial.begin(9600);
  pinMode (MOTOR_R_PIN, OUTPUT);
  pinMode (MOTOR_L_PIN, OUTPUT);
  
  //Configure SPI communication
  SPI.begin();
  SPI.setDataMode(3);
  pinMode (SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, 1);
  
  //Zero Accelerometer
  /*
  digitalWrite(SS_PIN, 0); //Begin Trasmission
  SPI.transfer(SPI_READ&SPI_MULTIBYTE&ACCEL_DATA); //Send Address
  delay(1);
  xDelta = SPI.transfer(SPI_IDLE); //Read X
  xDelta |= SPI.transfer(SPI_IDLE) << 8;
  yDelta = SPI.transfer(SPI_IDLE); //Read y
  yDelta |= SPI.transfer(SPI_IDLE) << 8;
  digitalWrite(SS_PIN, 1); //End transmission
  delay(1);
  digitalWrite(SS_PIN, 0); //Begin Trasmission
  delay(1);
  SPI.transfer(SPI_WRITE&SPI_MULTIBYTE&ACCEL_OFFSET); //Send Address
  SPI.transfer((signed char)-xDelta); //Write x offset
  SPI.transfer((signed char)-yDelta); //Write y offset
  digitalWrite(SS_PIN, 1); //End transmission
  delay(1);
  */
  digitalWrite(SS_PIN, 0); //Begin Trasmission
  SPI.transfer(SPI_WRITE&DATA_FORMAT); //Write to power controller
  SPI.transfer(0x01); //Turn off powersave mode
  digitalWrite(SS_PIN, 1); //End transmission
  delay(1);
  digitalWrite(SS_PIN, 0); //Begin Trasmission
  SPI.transfer(SPI_WRITE&0x2D); //Write to power controller
  SPI.transfer(0x08); //Turn off powersave mode
  digitalWrite(SS_PIN, 1); //End transmission
  //Zero Gyro
  headingOffset = -86;
  
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
  moveCommand = MOVE_FORWARD;
  delay(2000);
  moveCommand = 0;
  while(1);
}


/*
  Sample code from http://www.brainstemeducation.com/arduino_jaguar
  Sample code from https://bitbucket.org/kc8nod/arduino-jaguar
*/

