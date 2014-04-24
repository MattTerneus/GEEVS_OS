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
#define MOVE_TURN 1
#define MOVE_FORWARD 2
#define IDLE_SPEED 47
#define BASE_SPEED_R 7
#define BASE_SPEED_L 8
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
volatile char interruptCount, interruptFlag = 0;

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
  delta = analogRead(0) + headingOffset; //Read 10 bit ADC
  if (abs(delta) < 5)
  {
     delta = 0; 
  }
  testGyro = delta;
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
    if (pingCenter.centimeters() >= 100) //Only move if there is room ahead
    {
      newSpeedR = IDLE_SPEED - BASE_SPEED_R;
      newSpeedL = IDLE_SPEED + BASE_SPEED_L;
      if (pingCenter.centimeters() < 150) //Slow robot near obstacles
      {
        newSpeedR = IDLE_SPEED - (BASE_SPEED_R>>1);
        newSpeedL = IDLE_SPEED + (BASE_SPEED_L>>1);
      }
    }
    analogWrite(MOTOR_R_PIN,newSpeedR);
    analogWrite(MOTOR_L_PIN,newSpeedL);
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
      newSpeedR += BASE_SPEED_R>>2;
    else if (headingDelta < -ROTATION_DEADZONE)
      newSpeedR -= BASE_SPEED_R>>2;
         
    analogWrite(MOTOR_R_PIN,newSpeedR);
    analogWrite(MOTOR_L_PIN,newSpeedR);
  }
  else //Default mode
  {
    analogWrite(MOTOR_R_PIN,IDLE_SPEED);
    analogWrite(MOTOR_L_PIN,IDLE_SPEED);
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
  headingOffset = -504;
  
  //Setup real time services
  for (int i = 0; i < ROUTINES_PER_CYCLE; i++)
    routines[i] = 0;
    
  routines[0] = &accelerometer_service;
  routines[1] = &gyro_service;
  routines[2] = &ping_service;
  routines[3] = &motor_service;
  
  //Begin real time clock
  init_interrupt_timer();
  //Timer1.attachInterrupt(realtime_service_call);
  //Timer1.initialize(INTERRUPT_PERIOD);
}

void loop()
{
  realtime_service_call();
  moveCommand = MOVE_TURN;
  
  Serial.println(headingDeg);
  //Serial.println(headingDeg);
  //Serial.println(testGyro);
  //analogWrite(MOTOR_R_PIN,IDLE_SPEED+BASE_SPEED);
  //analogWrite(MOTOR_L_PIN,IDLE_SPEED+BASE_SPEED);
  //delay(2000);
  //analogWrite(MOTOR_R_PIN,IDLE_SPEED);
  //analogWrite(MOTOR_L_PIN,IDLE_SPEED);
  //moveCommand = 0;
  //while(1);
}


/*
  Sample code from http://www.brainstemeducation.com/arduino_jaguar
  Sample code from https://bitbucket.org/kc8nod/arduino-jaguar
*/

