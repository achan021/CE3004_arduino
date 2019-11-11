  //Ways to improve hardware performance:
//1) increase accuracy of IR sensors by increasing the sampling size
//2) Change the stabilise time to adjust the 10cm movement of the robot
//3) Full brakes for right and left turn (implement slow stop if needed)

//things left to do
//1) PID values tuning
//2) Tuning to the maze board (move_start, move_stop, turn_left, turn_right, move_hold)
//3) calibration code
//4) instruction from RPI code (Serial communication)
//5) tune PID
//6) calibration need to use double and mini threshold. integer reading causes rounding error and slight tilt

#include "DualVNH5019MotorShield.h"
#include "Encoder.h"
#include "PinChangeInt.h"
#include "Arduino.h"
#include "string.h"
#include "PID_v1.h"

DualVNH5019MotorShield md;

//*****Pin allocation
//Sharp IR sensor
#define PS1 A0
#define PS2 A1
#define PS3 A2
#define PS4 A3
#define PS5 A4
#define PS6 A5
//Motor encoder
#define e2a 11
#define e2b 13
#define e1a 3
#define e1b 5

#define right 1
#define left -1

volatile int counterA = 0;
volatile int counterB = 0;
double prevTick = counterA;
volatile double counterC = 0;
double difference;
double Setpoint = 0, Input, Output;
//PID set for exploration && image recognition
double Ku = 3.3 , Tu = 0.45;
double Kp = 0.6*Ku, Ki = 1.2*Ku/Tu, Kd = 3*Ku*Tu/40;
//Ku = 3.7 , Tu = 0.41;
PID mPID(&Input,&Output,&Setpoint,Kp,Ki,Kd,DIRECT);
//PID set for fastest path
double Kuf = 3.4, Tuf = 0.39;
double Kpf = 0.6*Kuf, Kif = 1.2*Kuf/Tuf, Kdf = 3*Kuf*Tuf/40;
PID mfPID(&Input,&Output,&Setpoint,Kpf,Kif,Kdf,DIRECT);
double quick = 25;
//20

double sensor1,sensor2,sensor3,sensor4,sensor5,sensor6;
int move_step;

void setup() {
  Serial.begin(115200);
  //Allocate the analog pins on the arduino and set them as input pins
  pinMode(PS1,INPUT);
  pinMode(PS2,INPUT);
  pinMode(PS3,INPUT);
  pinMode(PS4,INPUT);
  pinMode(PS5,INPUT);
  pinMode(PS6,INPUT);

  //Interrupt the arduino to update the encoder counters at the falling edge of the encoder reading
  PCintPort::attachInterrupt(e1a,outputAInc,RISING);
  PCintPort::attachInterrupt(e2a,outputBInc,RISING);
  md.init();
  Serial.setTimeout(60);
}

char command_buffer[50];
int i = 0 ;
char readChar;
bool complete = false;
char* command;
void loop() {
   
   /*---------------------------------------------------------------------------------------------------
                                 Cordless calibration (comment main code)
  ---------------------------------------------------------------------------------------------------*/
   //Current voltage 6.218
   //6.157
   //First check : turning left and right
   /*
   //Left turn check
   turnLeft(90);
   delay(2000);
   turnLeft(90);
   delay(2000);
   turnLeft(90);
   delay(2000);
   turnLeft(90);
   delay(2000);
   turnLeft(90);
   delay(2000);
   turnLeft(90);
   delay(2000);
   turnLeft(90);
   delay(2000);
   turnLeft(90);
   delay(3000);
   //Right turn check
   turnRight(90);
   delay(2000);
   turnRight(90);
   delay(2000);
   turnRight(90);
   delay(2000);
   turnRight(90);
   delay(2000);
   turnRight(90);
   delay(2000);
   turnRight(90);
   delay(2000);
   turnRight(90);
   delay(2000);
   turnRight(90);
   delay(30000);
   */

   //forward movement test
   /*
   turnLeft(90);
   full_calibration();
   turnRight(90);
   delay(400);
   //move hold 5 steps, simulate run
   move_hold(1);
   delay(400);
   move_hold(1);
   delay(400);
   move_hold(1);
   delay(400);
   move_hold(1);
   delay(400);
   move_hold(1);
   delay(400);
   move_hold(1);
   delay(400);
   move_hold(1);
   delay(400);
   move_hold(1);
   delay(2000);
   //move hold test
   turnRight(90);
   delay(2000);
   turnRight(90);
   delay(2000);
   move_hold(8);
   delay(30000);
   */

   //fastest path forward movement test
   /*
   turnLeft(90);
   full_calibration();
   turnRight(90);
   delay(200);
   move_hold_fast(7);
   delay(200);
   turnRight(90);
   delay(200);
   turnRight(90);
   delay(200);
   move_hold_fast(7);
   delay(200);
   turnLeft(90);
   delay(200);
   turnLeft(90);
   delay(30000);
   */
   
   
  /*---------------------------------------------------------------------------------------------------
                                 Establishing Serial Connection with RPi
  ---------------------------------------------------------------------------------------------------*/
  
  for (int i = 0; i < 10 ; i++){
    command_buffer[i] = NULL;  
  }
  if (Serial.available()){
    Serial.readBytes(command_buffer,50);
    command = strtok(command_buffer,",");
    //[command][distance][distance]
    while (command != NULL){
      String scommand = "";
      int arg = 0;
      for (int i = 1; i < 4; i++){
        scommand += command[i];  
      }
      arg = scommand.toInt();
      
      
//      Command Legends:
      
//      W ---> Move Forward
//      A ---> Rotate Left
//      D ---> Rotate Right
//      B ---> Break
//      S ---> Read Sensor Values
//      F ---> full calibration
//      Q ---> Quick calibration (no side dist check)
     
     
      switch ( command[0] ) {
        case 'W':
          { 
            if (arg == 0) move_hold(1);
            else move_hold(arg);
            Serial.println("MS");
            break;
          }
        case 'A':
          {      
            turnLeft(90);
            Serial.println("LD");
            break;
          }
        case 'D':
          {
            turnRight(90);
            Serial.println("RD");
            break;
          }
        //180 turn addition
        case 'K':
          {
            turnRight(180);
            Serial.println("KD");
            break;
          }
        case 'B':
        {
          md.setBrakes(400, 400);
          Serial.println("BR");
          break;
        }
        case 'S':
          {
            sensorSense(false);
            break;
          }
        case 'F':
          {
            full_calibration();
            Serial.println("CD");
            break;
          }
        case 'Q':
          {
            calibration();
            calibration();
            Serial.println("QD");
            break;
          }
        case 'R':
          {
            Serial.println();
            break;
          }
        case 'I':
          {
            if (arg == 0) move_hold_fast(1);
            else move_hold_fast(arg);
            Serial.println("MS");
            break;
          }
         case 'J':
          {
            turnLeft_fast(90);
            Serial.println("LD");
            break;
          }
         case 'L':
          {
            turnRight_fast(90);
            Serial.println("RD");
            break;
          }
      }
    delay(90);
    command = strtok(NULL,",");
    
    }
  }
}

  


bool stringRx;
String stringMsg;


//*************************mergeSorting algorithm*******************//

void merge(double arr[], int l, int m, int r) 
{ 
    int i, j, k; 
    int n1 = m - l + 1; 
    int n2 =  r - m; 
  
    /* create temp arrays */
    int L[n1], R[n2]; 
  
    /* Copy data to temp arrays L[] and R[] */
    for (i = 0; i < n1; i++) 
        L[i] = arr[l + i]; 
    for (j = 0; j < n2; j++) 
        R[j] = arr[m + 1+ j]; 
  
    /* Merge the temp arrays back into arr[l..r]*/
    i = 0; // Initial index of first subarray 
    j = 0; // Initial index of second subarray 
    k = l; // Initial index of merged subarray 
    while (i < n1 && j < n2) 
    { 
        if (L[i] <= R[j]) 
        { 
            arr[k] = L[i]; 
            i++; 
        } 
        else
        { 
            arr[k] = R[j]; 
            j++; 
        } 
        k++; 
    } 
  
    /* Copy the remaining elements of L[], if there 
       are any */
    while (i < n1) 
    { 
        arr[k] = L[i]; 
        i++; 
        k++; 
    } 
  
    /* Copy the remaining elements of R[], if there 
       are any */
    while (j < n2) 
    { 
        arr[k] = R[j]; 
        j++; 
        k++; 
    } 
}

void mergeSort(double arr[], int l, int r) 
{ 
    if (l < r) 
    { 
        // Same as (l+r)/2, but avoids overflow for 
        // large l and h 
        int m = l+(r-l)/2; 
  
        // Sort first and second halves 
        mergeSort(arr, l, m); 
        mergeSort(arr, m+1, r); 
  
        merge(arr, l, m, r); 
    } 
} 


//********************Getter functions*********************//

double getSensorDistance(int sensorNumber){
  return(sensorSelector(sensorNumber));
  
}


//***********************CALIBRATION CODE******************//

//*************************SENSORS**************************//

//Get the Va2d value 
double sensorSelector(int sensorNumber){
  int Va2d;
  double actualDist;
  int samples = 100;
  double sample_array[samples];
   
  switch(sensorNumber){
    case 1:
        for (int i = 0; i < samples; i ++){
          Va2d = analogRead(PS1);
          //determined from linearisation
          actualDist = 1/(0.000236877117527*Va2d - 0.013664629527719);
          sample_array[i] = actualDist;  
        }
        mergeSort(sample_array, 0, samples-1);
        //print the median distance measured.
        actualDist = sample_array[(samples+1)/2];
        break;
    case 2:
        for (int i = 0; i < samples; i ++){
          Va2d = analogRead(PS2);
          //determined from linearisation
          actualDist = 1/(0.000205292561516*Va2d - 0.002979440089537);
          sample_array[i] = actualDist;  
        }
        mergeSort(sample_array, 0, samples-1);
        //print the median distance measured.
        actualDist = sample_array[(samples+1)/2];
        break;
    case 3: 
        for (int i = 0; i < samples; i ++){
          Va2d = analogRead(PS3);
          //determined from linearisation
          actualDist = 1/(0.000237927563197*Va2d - 0.008493537114227) + 0.5;
          sample_array[i] = actualDist;  
        }
        mergeSort(sample_array, 0, samples-1);
        actualDist = sample_array[(samples+1)/2];
        //print the median distance measured.
        break;
    case 4:
        for (int i = 0; i < samples; i ++){
          Va2d = analogRead(PS4);
          //determined from linearisation
          actualDist = 1/(0.000196937162468*Va2d - 0.003860894333097) - 1;
          sample_array[i] = actualDist;  
        }
        mergeSort(sample_array, 0, samples-1);
        //print the median distance measured.
        actualDist = sample_array[(samples+1)/2];
        break;
    case 5:
        for (int i = 0; i < samples; i ++){
          Va2d = analogRead(PS5);
          //determined from linearisation
          actualDist = 1/(0.000207933644901*Va2d - 0.004400445451992) -0.3;
          sample_array[i] = actualDist;  
        }
        mergeSort(sample_array, 0, samples-1);
        //print the median distance measured.
        actualDist = sample_array[(samples+1)/2];
        break;
    case 6:
        for (int i = 0; i < samples; i ++){
          Va2d = analogRead(PS6);
          //determined from linearisation
          actualDist = 1/(0.000048404138075*Va2d + 0.002687355664031) - 15;
          sample_array[i] = actualDist;  
        }
        mergeSort(sample_array, 0, samples-1);
        //print the median distance measured.
        actualDist = sample_array[(samples+1)/2];
        break;
    default:
        //Serial.println("Wrong Selection");
        actualDist = 0;
        break;  
  }
  
  return actualDist;
  
}


void outputAInc(void){
  counterA++; 
  counterC++;
}

void outputBInc(void){
  counterB++;
}

//return the rounded reading of sensors. 10 cm -> 1 , 20cm -> 2 and 30cm -> 3 and else -> -1. This means that we limit out robot's range to only 30cm.
int returnRounded(double distance){
  if (distance < 10 && distance >= 0){
    return 1;  
  }
  else if (distance >= 10 && distance < 20){
    return 2;
  }
  else if (distance >= 20 && distance < 30){
    return 3;
  }
  
  else
    return -1;
  
}

void sensorSense(bool movement){
    
    //50 samples take about 54 millis to read (must be less than sampling time of 100)
    //distance from the boundary of the robot
    //distance from the boundary to the next step
     
//      sensor1 = getSensorDistance(1)-5.3;
//      sensor5 = getSensorDistance(5)-4.3;
//      sensor2 = getSensorDistance(2)-5.6;
//      sensor3 = getSensorDistance(3)-7;
//      sensor4 = getSensorDistance(4)-5.3;
//      sensor6 = getSensorDistance(6)-11;
//      
      sensor1 = returnRounded(getSensorDistance(1)-5.3);
      sensor5 = returnRounded(getSensorDistance(5)-4.3);
      sensor2 = returnRounded(getSensorDistance(2)-5.6);
      sensor3 = returnRounded(getSensorDistance(3)-7);
      sensor4 = returnRounded(getSensorDistance(4)-5.3);
      sensor6 = returnRounded(getSensorDistance(6)-11);

      
      //distance "(S1)(S2)(S3)(S4)(S5)(S6)" concatenated and send to the RPI then to algo, Algo will split by "," to get the readings of each sensor.
      String sensorVal = String(sensor1)+","+String(sensor2)+","+String(sensor3)+","+String(sensor4)+","+String(sensor5)+","+String(sensor6);
      int len = sensorVal.length();
      char char_array[len+1];
      sensorVal.toCharArray(char_array,len);
      Serial.println(char_array);

    
    /*
     *         (Back)
     *          ---x x x (s1)
     *          ||| 
     * (s6)x x x---x x x (s5)
     *       x   x   x
     *       x   x   x
     *       x   x   x
     *      (s4)(s3)(s2)
     *        (Front)
     */     
}



//PID is only set at move_hold because you only can tune when moving straight
void move_hold(int steps){
    int tempDetect[steps][2];
    int count = 0;
    int pid;
    //All these are in RPM values
    int forwardClimb = 110;
    //initial ramp RPM
    int interS1 = 90;
    //stopping ramp RPM
    int interE1 = 70;
    counterA = 0;
    counterB = 0;
    counterC = 0;

    mPID.SetMode(AUTOMATIC);
    mPID.SetSampleTime(100);
    mPID.SetOutputLimits(-400,400);
    //number of ticks per step
    int target_tick = 280;
    //There is a linear offset error based on the number of steps. (Measured during the checklist phase)
    double offset_error = 0.05*(steps*10)-0.5;
    int offset = (562.25/(6*PI))*offset_error;
    int loopCount = counterC;
    //Amount of ticks for initial ramp and stopping ramp -> secdelayTicks
    int secdelayTicks = 30;
    
    // + offset
    while (counterC < (target_tick * steps + offset - 14)){
      double cTime = micros();
      double delayS = micros(), delayE = micros();
      while(delayE - delayS < 5000){
        delayE = micros();
      }
      //input of PID is the difference between the 2 tick count of the motors.
      Input = counterA - counterB;
      bool compute = mPID.Compute();
      //quick is the manual offset to lower the speed of the motor1. Motor1 is ultimately faster therefore, causing robot to right drift.
      //initial ramp
      if (counterC < secdelayTicks && compute)
        md.setSpeeds(right*spd_motor1(interS1)+Output-quick,left*(spd_motor2(interS1)-Output));//270
      //stopping ramp
      else if (counterC >=secdelayTicks && counterC <= (target_tick * steps) - secdelayTicks && compute )
        md.setSpeeds(right*spd_motor1(forwardClimb)+Output-quick,left*(spd_motor2(forwardClimb)-Output));//270
      else if ( counterC > (target_tick * steps) - secdelayTicks && compute)
        md.setSpeeds(right*spd_motor1(interE1)+Output-quick,left*(spd_motor2(interE1)-Output));//270
    }
    //reset counterC for next movement
    counterC = 0;
    md.setBrakes(400,400);
}

//Old code used for checklist clearing.
void move_hold_checklist(int stopPoint){

    int pid;
    //chg to RPM
    int forwardClimb = 70;
    int inter = 45;
    int inter2 = 30;
    counterA = 0;
    counterB = 0;
    counterC = 0;

    mPID.SetMode(AUTOMATIC);
    mPID.SetSampleTime(100);
    mPID.SetOutputLimits(-400,400);

    while (getSensorDistance(3) > stopPoint){
      
      Input = counterA - counterB;
      bool compute = mPID.Compute();
      if (counterC < 30 && compute)
        md.setSpeeds(right*spd_motor1(inter)+Output,left*(spd_motor2(inter)-Output));//270
      else if (counterC >=30 && compute )
        md.setSpeeds(right*spd_motor1(forwardClimb)+Output,left*(spd_motor2(forwardClimb)-Output));//270
        
      }
      
    counterC = 0;
    md.setBrakes(400,400);
}

//old code used for testing. Not implemented for actual robot movement.
void reverse_move_hold(double steps){

    int pid;
    //chg to RPM
    int forwardClimb = 115;
    int inter = 80;
    int inter2 = 40;
    counterA = 0;
    counterB = 0;
    counterC = 0;

    mPID.SetMode(AUTOMATIC);
    mPID.SetSampleTime(100);
    mPID.SetOutputLimits(-400,400);
    
    int target_tick = 297;
    while (counterC < target_tick * steps){
      
      Input = counterA - counterB;
      
      bool Compute = mPID.Compute();

      if (counterC < 30 && Compute)
        md.setSpeeds(left*(spd_motor1(inter)+Output),right*(spd_motor2(inter)-Output));//270
      else if (counterC > (target_tick * steps) - 30 && Compute)
        md.setSpeeds(left*(spd_motor1(inter2)+Output),right*(spd_motor2(inter2)-Output));//270
      else if (counterC >= 30 && counterC <= (target_tick * steps) - 30 && Compute)
        md.setSpeeds(left*(spd_motor1(forwardClimb)+Output),right*(spd_motor2(forwardClimb)-Output));//270
      
    }
    
    
    counterC = 0;
    md.setBrakes(400,400);
}

void turnLeft(double angle){
    int forwardClimb = 100;
    double inter = 80,inter2 = 60;
    counterA = 0;
    counterB = 0;
    counterC = 0;
    mPID.SetMode(AUTOMATIC);
    mPID.SetSampleTime(100);
    mPID.SetOutputLimits(-400,400);
    //number of ticks required to turn the robot 90 degrees left. -13 is the manual tuning offset based on actual run days 
    int turnTicks = (int)(((562.25 * 17/6)/360) * (angle))-13;
    //+ 0.0278*angle this was initial offset for turning more than 90 degrees, for checklist
    //This delay is employed to try and mitigate the 45 degrees error occasionally done by the robot (Possibly hardware problem)
    double delayS = micros(), delayE = micros();
      while(delayE - delayS < 5000){
        delayE = micros();
      }
    while(counterC < turnTicks){
      Input = counterA - counterB;
      bool Compute = mPID.Compute();
      //10 degrees of ramp and 10 of deramp, about 40 ticks
      if (counterC < 40 && Compute)
        md.setSpeeds(left*(spd_motor1(inter)+Output),left*(spd_motor2(inter)-Output));//270
      else if (counterC > (turnTicks - 40) && Compute)
        md.setSpeeds(left*(spd_motor1(inter2)+Output),left*(spd_motor2(inter2)-Output));//270
      else if (counterC >= 40 && counterC <= (turnTicks) - 40 && Compute)
        md.setSpeeds(left*(spd_motor1(forwardClimb)+Output),left*(spd_motor2(forwardClimb)-Output));
    }
    
    md.setBrakes(400 ,400);
    counterC = 0;
}

void turnRight(double angle){
    
    int forwardClimb = 100;
    double inter = 80,inter2 = 60;
    counterA = 0;
    counterB = 0;
    counterC = 0;
    mPID.SetMode(AUTOMATIC);
    mPID.SetSampleTime(100);
    mPID.SetOutputLimits(-400,400);
    //number of ticks required to turn the robot 90 degrees right. -16 is the manual tuning offset based on actual run days 
    int turnTicks = (int)(((562.25 * 17/6)/360) * (angle))-16;
    //last try is 17
    //This delay is employed to try and mitigate the 45 degrees error occasionally done by the robot (Possibly hardware problem)
    double delayS = micros(), delayE = micros();
      while(delayE - delayS < 5000){
        delayE = micros();
      }
    while(counterC < turnTicks){
      Input = counterA - counterB;
      bool Compute = mPID.Compute();
      //10 degrees of ramp and 10 of deramp, about 40 ticks
      if (counterC < 40 && Compute)
        md.setSpeeds(right*(spd_motor1(inter)+Output),right*(spd_motor2(inter)-Output));//270
      else if (counterC > (turnTicks - 40) && Compute)
        md.setSpeeds(right*(spd_motor1(inter2)+Output),right*(spd_motor2(inter2)-Output));//270
      else if (counterC >= 40 && counterC <= (turnTicks) - 40 && Compute)
        md.setSpeeds(right*(spd_motor1(forwardClimb)+Output),right*(spd_motor2(forwardClimb)-Output));
      
    }
    
    md.setBrakes(400 ,400);
    counterC = 0;
}
/****************     Fastest path code  **********************/

void move_hold_fast(int steps){
    int count = 0;
    int pid;
    //chg to RPM
    //test is the speed reduction on the motor 1 to reduce right drift (same as quick in normal movement)
    int test = 25;
    int forwardClimb = 140;
    int inter = 100;
    int inter2 = 40;
    counterA = 0;
    counterB = 0;
    counterC = 0;

    mfPID.SetMode(AUTOMATIC);
    mfPID.SetSampleTime(100);
    mfPID.SetOutputLimits(-400,400);
    
    int target_tick = 280;
    double offset_error = 0.05*(steps*10)-0.5;
    int offset = (562.25/(6*PI))*offset_error;
    int loopCount = counterC;
    
    
    while (counterC < (target_tick * steps) + offset - 8){
      double cTime = micros();
      Input = counterA - counterB;
      bool compute = mPID.Compute();
      if (counterC < 30 && compute)
        md.setSpeeds(right*spd_motor1(inter)+Output-test,left*(spd_motor2(inter)-Output));//270
      else if ( counterC > (target_tick * steps) - 30 && compute)
        md.setSpeeds(right*spd_motor1(inter2)+Output-test,left*(spd_motor2(inter2)-Output));//270
      else if (counterC >=30 && counterC <= (target_tick * steps) - 30 && compute ){
        md.setSpeeds(right*spd_motor1(forwardClimb)+Output-test,left*(spd_motor2(forwardClimb)-Output));//270
  
      }
      
    }
    
    counterC = 0;
    md.setBrakes(400,400);
}

//fast turn left is not employed bcse it introduces new tuning issues. Therefore, we make the turn slow but forward movement fast instead.
void turnLeft_fast(double angle){
    int forwardClimb = 140;
    double inter = 100,inter2 = 40;
    counterA = 0;
    counterB = 0;
    counterC = 0;
    mfPID.SetMode(AUTOMATIC);
    mfPID.SetSampleTime(100);
    mfPID.SetOutputLimits(-400,400);
    int turnTicks = (int)(((562.25 * 17/6)/360) * (angle + 0.0278*angle));
    
    while(counterC <turnTicks){
      Input = counterA - counterB;
      bool Compute = mPID.Compute();
      //10 degrees of ramp and 10 of deramp, about 40 ticks
      if (counterC < 40 && Compute)
        md.setSpeeds(left*(spd_motor1(inter)+Output),left*(spd_motor2(inter)-Output));//270
      else if (counterC > (turnTicks - 40) && Compute)
        md.setSpeeds(left*(spd_motor1(inter2)+Output),left*(spd_motor2(inter2)-Output));//270
      else if (counterC >= 40 && counterC <= (turnTicks) - 30 && Compute)
        md.setSpeeds(left*(spd_motor1(forwardClimb)+Output),left*(spd_motor2(forwardClimb)-Output));
    }
    
    md.setBrakes(400 ,400);
    counterC = 0;
}
//fast turn right is not employed bcse it introduces new tuning issues. Therefore, we make the turn slow but forward movement fast instead.
void turnRight_fast(double angle){
    
    int forwardClimb = 140;
    double inter = 100,inter2 = 40;
    counterA = 0;
    counterB = 0;
    counterC = 0;
    mfPID.SetMode(AUTOMATIC);
    mfPID.SetSampleTime(100);
    mfPID.SetOutputLimits(-400,400);
    
    int turnTicks = (int)(((562.30 * 17/6)/360) * (angle + 0.0278*angle)) ;
    
    while(counterC < turnTicks){
      Input = counterA - counterB;
      bool Compute = mPID.Compute();
      //10 degrees of ramp and 10 of deramp, about 40 ticks
      if (counterC < 40 && Compute)
        md.setSpeeds(right*(spd_motor1(inter)+Output),right*(spd_motor2(inter)-Output));//270
      else if (counterC > (turnTicks - 40) && Compute)
        md.setSpeeds(right*(spd_motor1(inter2)+Output),right*(spd_motor2(inter2)-Output));//270
      else if (counterC >= 40 && counterC <= (turnTicks) - 30 && Compute)
        md.setSpeeds(right*(spd_motor1(forwardClimb))+Output,right*(spd_motor2(forwardClimb)-Output));
      
    }
    
    md.setBrakes(400 ,400);
    counterC = 0;
}

/****************     Calibration code   **********************/
//robot will creep forward
void move_hold_noRamp(){

    //chg to RPM
    int forwardClimb = 200;
    counterA = 0;
    counterB = 0;
    counterC = 0;

    mPID.SetMode(AUTOMATIC);
    mPID.SetSampleTime(100);
    mPID.SetOutputLimits(-400,400);
    
    int target_tick = 2;
    while (counterC < target_tick){
      
      Input = counterA - counterB;
      bool compute = mPID.Compute();
      if (compute)
      {
        md.setSpeeds(right*400+Output,left*(400-Output));//270
      }
    }
    
    counterC = 0;
    md.setBrakes(400,400);
}

//robot will creep backwards
void reverse_move_hold_noRamp(){

    int pid;
    //chg to RPM
    int forwardClimb = 200;
    counterA = 0;
    counterB = 0;
    counterC = 0;

    mPID.SetMode(AUTOMATIC);
    mPID.SetSampleTime(100);
    mPID.SetOutputLimits(-400,400); 
    
    int target_tick = 3;
    while (counterC < target_tick ){
      Input = counterA - counterB;
      
      if (mPID.Compute()){
         md.setSpeeds(left*(400+Output),right*((400)-Output));//270
      }
    }
    counterC = 0;
    md.setBrakes(400,400);
}
//robot will creep left to correct right tilt
void turnLeft_noRamp(double angle){
    int forwardClimb = 200;
    counterA = 0;
    counterB = 0;
    counterC = 0;
    mPID.SetMode(AUTOMATIC);
    mPID.SetSampleTime(100);
    mPID.SetOutputLimits(-400,400);
    int turnTicks = (int)(((562.25 * 17/6)/360) * (angle));
    
    while(counterC <turnTicks){
      Input = counterA - counterB;
      mPID.Compute();
      
      md.setSpeeds(left*(spd_motor1(forwardClimb))+Output,left*(spd_motor2(forwardClimb)-Output));
    }
    
    md.setBrakes(400 ,400);
    counterC = 0;
}

//robot will creep right to correct left tilt
void turnRight_noRamp(double angle){
    int forwardClimb = 200;
    counterA = 0;
    counterB = 0;
    counterC = 0;
    mPID.SetMode(AUTOMATIC);
    mPID.SetSampleTime(100);
    mPID.SetOutputLimits(-400,400);
    int turnTicks = (int)(((562.25 * 17/6)/360) * (angle));
    
    while(counterC <turnTicks){
      Input = counterA - counterB;
      mPID.Compute();
      
      md.setSpeeds(right*(spd_motor1(forwardClimb))+Output,right*(spd_motor2(forwardClimb)-Output));
    }
    
    md.setBrakes(400 ,400);
    counterC = 0;
}

//front and side calibration 
void full_calibration(){
  calibration();
  delay(200);
  turnLeft(90);
  delay(200);
  calibration();
  delay(200);
  turnRight(90);
  delay(200);
  
}

//abit of inefficient coding here bcse of time constraint
//calibration includes tilt check, distance check, then tilt check again.
void calibration(){
    
    //chg to RPM
    int calRPM = 100, samples = 5;
    counterA = 0;
    counterB = 0;
    counterC = 0;
   
    bool tiltCheck = false, distCheck = false;
    double angleInput,distInput,athreshold = 1,tthreshold = 0.2;
    int resetCounter = 0;
    
    
    //tiltCheck
    while (tiltCheck == false){
        if (getSensorDistance(3) -7 < 2.5)
          reverse_move_hold(0.25);
        angleInput = (getSensorDistance(4) -5.3) - (getSensorDistance(2) -6.1);
        
        //Serial.println(angleInput);


        
        //right tilt
        if (angleInput > 0 + athreshold){
          turnLeft_noRamp(0.5);
        }
        //left tilt
        else if ( angleInput < 0){
          turnRight_noRamp(0.5);
        }
        else if (angleInput >= -1*athreshold && angleInput <= athreshold){
          delay(30);
          tiltCheck = true;
        }
        delay(40);
        resetCounter ++;
    }
    while(distCheck == false){
        
        distInput = getSensorDistance(3) -7;
        //Serial.println(distInput);
        //Serial.println(distInput);
        if (distInput > 5 + tthreshold){
          move_hold_noRamp();
        }  
        //move back
        else if (distInput < 5 - tthreshold){
          reverse_move_hold_noRamp();
          tiltCheck = false;
        }
        else if (distInput >= 5 - tthreshold && distInput <= 5 + tthreshold){
          delay(30);
          distCheck = true;
        }
        delay(30);
    }
    tiltCheck = false;
    while (tiltCheck == false){
        if (getSensorDistance(3) -7 < 2.5)
          reverse_move_hold(0.25);
        angleInput = (getSensorDistance(4) -5.3) - (getSensorDistance(2) -6.1);
        
        //Serial.println(angleInput);


        
        //right tilt
        if (angleInput > 0 + athreshold){
          turnLeft_noRamp(0.5);
        }
        //left tilt
        else if ( angleInput < 0){
          turnRight_noRamp(0.5);
        }
        else if (angleInput >= -1*athreshold && angleInput <= athreshold){
          delay(30);
          tiltCheck = true;
        }
        delay(40);
        resetCounter ++;
    }
}

//Conversion obtained from plotting the RPM vs Speed graph on excel.
//converts RPM to speed 
double spd_motor1(double RPM){
  if (RPM == 0){
    return 0;
  }
  return((RPM + 6.66877416698447000000)/ 0.40957074863028700000 );
}
//converts RPM to speed
double spd_motor2(double RPM){
  if (RPM == 0){
    return 0;
  }
  return((RPM + 5.07915683483009000000)/0.37142214919228400000 );
}

