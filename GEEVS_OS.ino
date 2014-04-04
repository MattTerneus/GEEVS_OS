#include <TimerOne.h>
#include <Ping.h>
#include <SPI.h>
#define CYCLES_PER_SECOND 4
#define ROUTINES_PER_CYCLE 4
#define INTERRUPT_PERIOD 1000000/(CYCLES_PER_SECOND*ROUTINES_PER_CYCLE)

void (*routines[ROUTINES_PER_CYCLE])();
int routineIndex = 0;
volatile int collisionDist, heading = 0;
volatile long xPos, yPos, xVel, yVel = 0;

void read_accelerometer ()
{
  
}

void read_gyro ()
{
  
}

void read_ping ()
{
  
}

void write_motors ()
{
  
}

void realtime_call ()
{
  if (routines[routineIndex] != 0)
    routines[routineIndex]();
  routineIndex++;
  if (routineIndex == ROUTINES_PER_CYCLE)
    routineIndex = 0;
}

void setup ()
{
  for (int i = 0; i < ROUTINES_PER_CYCLE; i++)
    routines[i] = 0;
    
  routines[0] = &read_accelerometer;
  routines[1] = &read_gyro;
  routines[2] = &read_ping;
  routines[3] = &write_motors;
  
  Timer1.attachInterrupt(realtime_call);
  Timer1.initialize(INTERRUPT_PERIOD);
  
}

void loop()
{
  
}

