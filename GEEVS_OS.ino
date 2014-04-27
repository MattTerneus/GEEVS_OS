#include <Ping.h>
#include <TimerOne.h>
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
#define MOVE_STOP 0
#define MOVE_TURN 1
#define MOVE_FORWARD 2
#define MOVE_WAIT 4
#define MOVE_MEASURE 8
#define IDLE_SPEED 47
#define BASE_SPEED_R 7
#define BASE_SPEED_L 6
#define LEFT_TURN 7
#define RIGHT_TURN 7
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
//unsigned int commands[16];
//unsigned char commandID = 0;
unsigned char board[1024];
unsigned int target;
unsigned int myPos = 495; // x = 15, y = 15 coords. (Y*32)+X = position on the board


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
      moveCommand = MOVE_STOP;
      distanceReset = 1;
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
        newSpeedR = IDLE_SPEED + RIGHT_TURN;
        newSpeedL = IDLE_SPEED + LEFT_TURN;
      }
      else if (headingDelta < -ROTATION_DEADZONE)
      {
        newSpeedR = IDLE_SPEED - RIGHT_TURN;
        newSpeedL = IDLE_SPEED - LEFT_TURN;
      }
      else
      {
        //Manuever Complete
        moveCommand = MOVE_STOP;
        distanceReset = 1;
        moveComplete = 1;
      }
         
      analogWrite(MOTOR_R_PIN,newSpeedR);
      analogWrite(MOTOR_L_PIN,newSpeedL);
    }
    else if (moveCommand == MOVE_WAIT)
    {
      if (distanceReset == 1)
      {
        distanceTicks = 0;
        distanceReset = 0;  
      }
      
      if (distanceTicks >= goalPosition)
      {
        moveCommand = MOVE_STOP;
        distanceReset = 1;
        moveComplete = 1;
      }  
      analogWrite(MOTOR_R_PIN,IDLE_SPEED);
      analogWrite(MOTOR_L_PIN,IDLE_SPEED);
    }
    else if (moveCommand == MOVE_MEASURE)
    {
      pingLeft.fire();
      pingRight.fire();
      Serial.println(pingLeft.centimeters());
      Serial.println(pingRight.centimeters());
      moveCommand = MOVE_STOP;
      distanceReset = 1;
      moveComplete = 1;
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
  
  for(int i =0; i < 1024;i++)
  {
    board[i] = 0;
  }
  board[myPos] = 2;
  
  //Begin real time clock
  init_interrupt_timer();

 //commands[0] = MOVE_WAIT;
 //commands[1] = 32;
 //commands[2] = MOVE_FORWARD;
 //commands[3] = 38;
 //commands[4] = MOVE_WAIT;
 //commands[5] = 16;
 //commands[6] = MOVE_MEASURE;
 //commands[7] = 0;
 //commands[8] = MOVE_TURN;
 //commands[9] = 180;
 //commands[10] = MOVE_STOP;
 //commands[11] = 0;
 
 moveComplete = 1;
}

void loop()
{
  realtime_service_call();
  
  
  //goalPosition = 38;
  //moveCommand = MOVE_FORWARD;
  //goalHeading = 270;
  //moveCommand = MOVE_TURN;
  
  
  //If The robot is Stationary, It is OK to do path finding calculations
  if (moveComplete == 1)
  {
    //Ping the ultrasonic sensors and update the board
    board[myPos] = board[myPos] + 1; // update timestamp
    if (board[myPos] == 255) // wipe the board, explored too much in same place
    {
      for(int i =0; i< 1024; i++)
      {
        board[i] = 0;
      }
    }
    
    moveComplete = 0;
    moveCommand = MOVE_MEASURE;
    // While pinging the sensors, continue to do other interrupts
    while (moveComplete == 0)
    {
      realtime_service_call();
    }
    if (headingDeg > 340 || headingDeg < 20)//NORTH, UP, towards y = 0
    {
      if (pingLeft.centimeters() < 100 && (myPos%32 - 1) >= 0)//WALL to the WEST (same y, x-1) so subtract 1 from current position
      {
        board[myPos-1] = 1;
      }
      else if ((board[myPos-1] == 0 || board[myPos-1]==1) && (myPos%32 - 1) >= 0)
      {
        board[myPos-1] = 2;
      }
      
      if(pingRight.centimeters() < 100 && (myPos%32 + 1) <= 31)//WALL to the EAST (same y, x+1) so add 1 to current position
      {
        board[myPos+1] = 1;
      }
      else if ((board[myPos+1] == 0 || board[myPos+1]==1) && (myPos%32 + 1) <= 31)
      {
        board[myPos+1] = 2;
      }
      
      if (pingCenter.centimeters() < 100 && myPos-32 >=0)//WALL to the NORTH (same x, y-1) so subtract 32 from current position
      {
        board[myPos-32] = 1;
      }
      else if ((board[myPos-32] == 0 || board[myPos-32]==1) && myPos-32 >=0)
      {
        board[myPos-32] = 2;
      }
    } // end if (headingDeg > 340 || headingDeg < 20)//NORTH, UP, towards y = 0
    else if (headingDeg > 70 && headingDeg < 110)//EAST, RIGHT, towards x = 31
    {
      if (pingLeft.centimeters() < 100 && myPos-32 >=0)//WALL to the NORTH (same x, y-1) so subtract 32 from current position
      {
        board[myPos-32] = 1;
      }
      else if ((board[myPos-32] == 0 || board[myPos-32]==1) && myPos-32 >=0)
      {
        board[myPos-32] = 2;
      }
      
      if(pingRight.centimeters() < 100 && myPos+32 <=1023)//WALL to the SOUTH (same x, y+1) so add 32 to current position 
      {
        board[myPos+32] = 1;
      }
      else if ((board[myPos+32] == 0 || board[myPos+32]==1) && myPos+32 <=1023)
      {
        board[myPos+32] = 2;
      }
      
      if (pingCenter.centimeters() < 100 && (myPos%32 + 1) <= 31)//WALL to the EAST (same y, x+1) so add 1 to current position
      {
        board[myPos+1] = 1;
      }
      else if ((board[myPos+1] == 0 || board[myPos+1]==1) && (myPos%32 + 1) <= 31)
      {
        board[myPos+1] = 2;
      }
    } // end else if (headingDeg > 70 && headingDeg < 110)//EAST, RIGHT, towards x = 31
    else if (headingDeg >160 && headingDeg <200)//SOUTH, DOWN, towards y = 31
    {
      if (pingLeft.centimeters() < 100 && (myPos%32 + 1) <= 31)//WALL to the EAST (same y, x+1) so add 1 to current position
      {
        board[myPos+1] = 1;
      }
      else if ((board[myPos+1] == 0 || board[myPos+1]==1) && (myPos%32 + 1) <= 31)
      {
        board[myPos+1] = 2;
      }
      
      if(pingRight.centimeters() < 100 && (myPos%32 - 1) >= 0)//WALL to the WEST (same y, x-1) so subtract 1 from current position
      {
        board[myPos-1] = 1;
      }
      else if ((board[myPos-1] == 0 || board[myPos-1]==1) && (myPos%32 - 1) >= 0)
      {
        board[myPos-1] = 2;
      }
      
      if (pingCenter.centimeters() < 100 && myPos+32 <=1023)//WALL to the SOUTH (same x, y+1) so add 32 to current position
      {
        board[myPos+32] = 1;
      }
      else if ((board[myPos+32] == 0 || board[myPos+32]==1) && myPos+32 <=1023)
      {
        board[myPos+32] = 2;
      }
    } // end else if (headingDeg >160 && headingDeg <200)//SOUTH, DOWN, towards y = 31
    else if (headingDeg >250 && headingDeg < 290)//WEST, LEFT, towards x = 0
    {
      if (pingLeft.centimeters() < 100 && myPos+32 <=1023)//WALL to the SOUTH (same x, y+1) so add 32 to current position
      {
        board[myPos+32] = 1;
      }
      else if ((board[myPos+32] == 0 || board[myPos+32]==1) && myPos+32 <=1023)
      {
        board[myPos+32] = 2;
      }
      
      if(pingRight.centimeters() < 100 && myPos-32 >=0)//WALL to the NORTH (same x, y-1) so subtract 32 from current position
      {
        board[myPos-32] = 1;
      }
      else if ((board[myPos-32] == 0 || board[myPos-32]==1) && myPos-32 >=0)
      {
        board[myPos-32] = 2;
      }
      
      if (pingCenter.centimeters() < 100 && (myPos%32 - 1) >= 0)//WALL to the WEST (same y, x-1) so subtract 1 from current position
      {
        board[myPos-1] = 1;
      }
      else if ((board[myPos-1] == 0 || board[myPos-1]==1) && (myPos%32 - 1) >= 0)
      {
        board[myPos-1] = 2;
      }
    } // end else if (headingDeg >250 && headingDeg < 290)//WEST, LEFT, towards x = 0
    //find a new target (where board is #2) 
    unsigned char smallestDistance = 200;
    unsigned char tempDistance = 200;
    char myX = myPos%32;
    char myY = myPos/32;
    char tempX;
    char tempY;
    char targetX;
    char targetY;
    for (int i = 0; i < 1024; i++)
    {
      realtime_service_call();
      if(board[i] == 2)
      {
        tempX = i%32;
        tempY = i/32;
        tempDistance = abs(myX - tempX) + abs(myY - tempY);
        if (tempDistance < smallestDistance)
        {
          smallestDistance = tempDistance;
          target = i;
          targetX = tempX;
          targetY = tempY;
        }
      }
    }
    //If there are no #2's, clear the board and start over
    if (tempDistance == 200)//never found a target
    {
      // Loop over and start over completely
      for(int i =0; i< 1024; i++)
      {
        realtime_service_call();
        board[i] = 0;
      }
    }
    else //Generate a path to that target(manhattan distance based on board timestamp)
    {
      //if target cant be reached (surrounded by walls, mark it as a wall and dont make a path)
      if ((targetX - 1 >= 0 && board[(targetY*32)+(targetX-1)] == 1) || targetX-1 < 0)
      {
        if((targetX + 1 <= 31 && board[(targetY*32)+(targetX+1)] == 1) || targetX+1 > 31)
        {
          if((targetY + 1 <= 31 && board[((targetY+1)*32)+(targetX)] == 1) || targetY+1 > 31)
          {
            if((targetY - 1 >= 0 && board[((targetY-1)*32)+(targetX)] == 1) || targetY-1 < 0)//cannot reach target
            {
              board[target] = 1;
            }
          }
          
        }
      }
      
      if (board[target] == 2)//target is good to make a path to
      {
        unsigned char timeStamps[4];
        timeStamps[0] = 1; //WEST
        timeStamps[1] = 1; //EAST
        timeStamps[2] = 1; //NORTH
        timeStamps[3] = 1; //SOUTH
        char dir = 0;//defaults to WEST
        //bias towards the direction of the target
        
        //look at surrounding tiles and pick the tile with the smallest timestamp to go to (a higher timestamp is like a wall) and add it to the path until target is reached
        //if a tie, pick towards the target
        
        //look to the left(west)
        if (myX-1 >=0 && board[(myY*32) + (myX-1)] >= 2)
        {
          timeStamps[0] = board[(myY*32) + (myX-1)];
        }

        //look to the right(east)
        if (myX+1 <=31 && board[(myY*32) + (myX+1)] >=2)
        {
          timeStamps[1] = board[(myY*32) + (myX+1)];
        }

        //look above(north)
        if (myY-1 >=0 && board[((myY-1)*32)+myX] >=2)
        {
          timeStamps[2] = board[((myY-1)*32) + myX];
        }
        //look below(south)
        if (myY+1 >=0 && board[((myY+1)*32)+myX] >=2)
        {
          timeStamps[3] = board[((myY+1)*32) + myX];
        }
        

        for (int j = 0; j <3; j++)//find smallest time stamp that is not 1. If a tie, pick towards the target
        {
          if (timeStamps[j+1] < timeStamps[dir] && timeStamps[j+1] >=2)
          {
            dir = j+1;
          }
          else if (timeStamps[dir] == 1 && timeStamps[j+1] >=2)
          {
            dir = j+1;
          }
          else if(timeStamps[j+1] == timeStamps[dir] && timeStamps[j+1] >=2)// a TIE has occured
          {
            if(myX - targetX >0 && dir == 0)//target is WEST, choose west (0)
            {
              dir = 0;
            }
            else if(myX - targetX < 0 && j+1 == 1)//target is EAST, choose east (1)
            {
              dir = 1;
            }
            else if(myY - targetY > 0 && j+1 == 2)//target is NORTH, choose north (2)
            {
              dir = 2;
            }
            else if(myY - targetY < 0 && j+1 == 3)//target is SOUTH, choose south(2)
            {
              dir = 3;
            }
          } // end else if(timeStamps[j+1]==timeStamps[dir])
        }
        
        //determine which heading is the goal direction
        if(dir == 0)//WEST
        {
         goalHeading = 270; 
        }
        else if(dir == 1)//EAST
        {
          goalHeading = 90;
        }
        else if(dir == 2)//NORTH
        {
          goalHeading = 0;
        }
        else if(dir == 3)//SOUTH
        {
          goalHeading = 180;
        }
    //    Serial.print("Before Move Turn");
        //command stuff here
        //Turn to goal heading
        moveCommand = MOVE_TURN;
        moveComplete = 0;
        // While pinging the sensors, continue to do other interrupts
        while(moveComplete == 0)
        {
          realtime_service_call();
        }
  //      Serial.print("Before Wait");
        //Wait
        goalPosition = 16;
        moveCommand = MOVE_WAIT;
        moveComplete = 0;
        // While pinging the sensors, continue to do other interrupts
        while(moveComplete == 0)
        {
          realtime_service_call();
        }
//        Serial.print("Before Move Forward");

       

        //move into the tile
        if (pingCenter.centimeters() >100)
        {
          goalPosition = 48;
          moveCommand = MOVE_FORWARD;
          moveComplete = 0;
          // While pinging the sensors, continue to do other interrupts
          while(moveComplete == 0)
          {
            realtime_service_call();
          }
          
          //update position on board
          if(dir == 0)//WEST
          {
            myPos = myPos-1;
          }
          else if(dir == 1)//EAST
          {
            myPos = myPos+1;
          }
          else if(dir == 2)//NORTH
          {
            myPos = myPos - 32;
          }
          else if(dir == 3)//SOUTH
          {
            myPos = myPos + 32;
          }
        }
        
        //Serial.print("Before Move Wait");
        //Wait
        goalPosition = 16;
        moveCommand = MOVE_WAIT;
        moveComplete = 0;
        // While pinging the sensors, continue to do other interrupts
        while(moveComplete == 0)
        {
          realtime_service_call();
        }
        
        moveCommand = MOVE_TURN;
        moveComplete = 0;
        // While pinging the sensors, continue to do other interrupts
        while(moveComplete == 0)
        {
          realtime_service_call();
        }
  //      Serial.print("Before Wait");
        //Wait
        goalPosition = 16;
        moveCommand = MOVE_WAIT;
        moveComplete = 0;
        // While pinging the sensors, continue to do other interrupts
        while(moveComplete == 0)
        {
          realtime_service_call();
        }
        
      } // Determined the target was not walled off
    } // Target was found 
  } // moveComplete==1
  
  /* 
    if (moveComplete == 1)
    {
      moveComplete = 0;
      goalPosition = commands[commandID+1];
      goalHeading = commands[commandID+1];
      moveCommand = commands[commandID];
      commandID = commandID+2;
    }
  */
} // End of loop()


/*
  Sample code from http://www.brainstemeducation.com/arduino_jaguar
  Sample code from https://bitbucket.org/kc8nod/arduino-jaguar
*/

