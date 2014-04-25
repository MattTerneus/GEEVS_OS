#include <TimerOne.h>
#include <Ping.h>
#include <SPI.h>

//System Values
#define CYCLES_PER_SECOND 4
#define ROUTINES_PER_CYCLE 4
#define INTERRUPT_PERIOD 1000000/(CYCLES_PER_SECOND*ROUTINES_PER_CYCLE)

//Pin data
#define SS_PIN 7
#define MOTOR_R_PIN 10
#define MOTOR_L_PIN 9

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
#define GYRO_CYCLE 2048

//Motor data
#define MOVE_IDLE 0
#define MOVE_TURN 1
#define MOVE_FORWARD 2
#define IDLE_SPEED 47
#define BASE_SPEED_R 9
#define BASE_SPEED_L 9
#define ROTATION_DEADZONE 6


void (*routines[ROUTINES_PER_CYCLE])();
volatile signed int headingRaw = 0;
volatile unsigned int headingDeg, routineIndex = 0;
unsigned char pingLock, moveCommand = 0;
volatile signed int goalHeading = 0, headingOffset = 0;
signed long goalPosition = 0;
volatile unsigned long distanceTicks = 0;
volatile char interruptCount, interruptFlag, distanceReset, moveComplete = 0;
unsigned char currentSpeedR = IDLE_SPEED, currentSpeedL = IDLE_SPEED;
unsigned int commands[8];
unsigned char commandID = 0;

Ping pingLeft = Ping(2);
Ping pingCenter = Ping(3);
Ping pingRight = Ping(4);

void gyro_service ()
{
  int delta;
  delta = analogRead(0) + headingOffset; //Read 10 bit ADC
  if (abs(delta) < 5)
  {
     delta = 0; 
  }
  headingRaw += delta; //Adjust heading
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
  unsigned char newSpeedR = IDLE_SPEED;
  unsigned char newSpeedL = IDLE_SPEED;
  signed int headingDelta;
  
  if (moveCommand == MOVE_FORWARD) //Forward Mode
  {
    if (distanceReset == 1)
    {
      distanceTicks = 0;
      distanceReset = 0;  
    }
    
    if (distanceTicks < goalPosition)
    {
      if (pingCenter.centimeters() >= 100) //Only move if there is room ahead
      {
        newSpeedR = IDLE_SPEED - BASE_SPEED_R;
        newSpeedL = IDLE_SPEED + BASE_SPEED_L;
      }
      else
      {
        //obstacle entered path 
      }
    }
    else
    {
      //Manuever complete
      moveCommand = MOVE_IDLE;
      moveComplete = 1;
      
    }
    
    if (newSpeedR > currentSpeedR)
      currentSpeedR++;
    else if (newSpeedR < currentSpeedR)
      currentSpeedR--;
    if (newSpeedL > currentSpeedL)
      currentSpeedL++;
    else if (newSpeedL < currentSpeedL)
      currentSpeedL--;
      
    analogWrite(MOTOR_R_PIN,currentSpeedR);
    analogWrite(MOTOR_L_PIN,currentSpeedL); 
  }
  else 
  {
    distanceReset = 1;
    if (moveCommand == MOVE_TURN) //Turn Mode
    {
      headingDelta = goalHeading-headingDeg;
      // > 180 degree turn compensation
      if (headingDelta > 180)
        headingDelta = 180-headingDelta;
      else if (headingDelta < -180)
        headingDelta = -180-headingDelta;
        
      if (headingDelta > ROTATION_DEADZONE)
      {
        newSpeedR = IDLE_SPEED + BASE_SPEED_R;
        newSpeedL = IDLE_SPEED + BASE_SPEED_L;
      }
      else if (headingDelta < -ROTATION_DEADZONE)
      {
        newSpeedR = IDLE_SPEED - BASE_SPEED_R;
        newSpeedL = IDLE_SPEED - BASE_SPEED_L;
      }
      else
      {
        //Manuever Complete
        moveCommand = MOVE_IDLE;
        moveComplete = 1;
      }
         
      analogWrite(MOTOR_R_PIN,newSpeedR);
      analogWrite(MOTOR_L_PIN,newSpeedL);
    }
    else //Default mode
    {
      analogWrite(MOTOR_R_PIN,IDLE_SPEED);
      analogWrite(MOTOR_L_PIN,IDLE_SPEED);
    }
  }
}

void realtime_service_call ()
{
  if (interruptFlag == 1)
  {
    interruptFlag = 0;
    if (routines[routineIndex] != 0) //Only call routine if it exists
      routines[routineIndex]();
    routineIndex++; //Advance to next routine
    if (routineIndex == ROUTINES_PER_CYCLE) //Wrap around at end of cycle
      routineIndex = 0;
  }
}

void init_interrupt_timer ()
{
  TCCR0A = 0;
  TCCR0B = 0;
  TCNT0  = 0;
  OCR0A = 195;
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 1024 prescaler
  TCCR0B |= (1 << CS02) | (1 << CS00);   
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);  
}

ISR(TIMER0_COMPA_vect)
{
  if (interruptCount == 5)
  {
    interruptFlag = 1;
    interruptCount = 0;
    distanceTicks++;
  }
  interruptCount++;
}

void setup ()
{
  signed int xDelta, yDelta;
    
  Serial.begin(9600);
  pinMode (MOTOR_R_PIN, OUTPUT);
  pinMode (MOTOR_L_PIN, OUTPUT);
  
  TCCR1B = TCCR1B & 0b11111000 | 0x04;
  pinMode (MOTOR_R_PIN, OUTPUT);
  pinMode (MOTOR_L_PIN, OUTPUT);
  analogWrite(MOTOR_R_PIN,IDLE_SPEED);
  analogWrite(MOTOR_L_PIN,IDLE_SPEED);
  headingOffset = -504;
  
  //Setup real time services
  for (int i = 0; i < ROUTINES_PER_CYCLE; i++)
    routines[i] = 0;
    
  routines[0] = &gyro_service;
  routines[1] = &motor_service;
  routines[2] = &ping_service;
  routines[3] = &motor_service;
  
  //Begin real time clock
  init_interrupt_timer();

 commands[0] = MOVE_FORWARD;
 commands[1] = 38;
 commands[2] = MOVE_TURN;
 commands[3] = 270;
 commands[4] = MOVE_TURN;
 commands[5] = 180;
 commands[6] = MOVE_IDLE;
 commands[7] = 180;
 
 moveComplete = 1;
}

void loop()
{
  realtime_service_call();
  //goalPosition = 38;
  //moveCommand = MOVE_FORWARD;
  
  //goalHeading = 270;
  //moveCommand = MOVE_TURN;
  
  
  if (moveComplete == 1)
  {
    moveComplete = 0;
    goalPosition = commands[commandID+1];
    goalHeading = commands[commandID+1];
    moveCommand = commands[commandID];
    commandID = commandID+2;
  }
  
  
}


/*
  Sample code from http://www.brainstemeducation.com/arduino_jaguar
  Sample code from https://bitbucket.org/kc8nod/arduino-jaguar
*/

