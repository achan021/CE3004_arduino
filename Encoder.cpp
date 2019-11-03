#include "Arduino.h"
#include "Encoder.h"

static int countAnt1 = 0;
static int countAnt2 = 0;
static int countBnt1 = 0;
static int countBnt2 = 0;
static unsigned long lastTime1 = 0;
static unsigned long lastTime2 = 0;
static double pSpeed1 = 0;
static double pSpeed2 = 0;

Encoder::Encoder(unsigned char pinA, unsigned char pinB, unsigned char pinC, unsigned char pinD){
	pinA1 = pinA;
	pinB1 = pinB;
	pinA2 = pinC;
	pinB2 = pinD;
}

void Encoder::init(){
	pinMode(pinA1, INPUT);
	pinMode(pinB1, INPUT);
	pinMode(pinA2, INPUT);
	pinMode(pinB2, INPUT);
	count1 = 0;
	count2 = 0;
}

void Encoder::initialize(){
  countAnt1 = count1;
  countBnt1 = count1;
  countAnt2 = count2;
  countBnt2 = count2;
  lastTime1 = micros()/1000;
  lastTime2 = micros()/1000;
  
}

long double Encoder::getMotor1RPM(unsigned long time1){
//	int speed = (count1 - countAnt1) / (time1 - lastTime1);4
    
    double speed1 = ((double)(count1 - countBnt1) * 60000.0) / ((time1 - lastTime1) * 562.215);
    countBnt1 = count1;
    lastTime1 = time1;
    pSpeed1 = speed1;
    return speed1;
  
	
}

long double Encoder::getMotor2RPM(unsigned long time2){
  
  	double speed2 = ((double)(count2 - countBnt2) * 60000.0) / ((time2 - lastTime2) * 562.215);
    countBnt2 = count2;
  	lastTime2 = time2;
    pSpeed2 = speed2;
  	return speed2;
}

double Encoder::getMotor1Revs(){
	int res =  count1 - countAnt1;
	countAnt1 = count1;
	return res/562.215;
}

double Encoder::getMotor2Revs(){
	int res = count2 - countAnt2;
	countAnt2 = count2;
	return res/562.215;
}

void Encoder::rencoder1()  {
    count1++; 
}

void Encoder::rencoder2()  {
    count2++;
}

int Encoder::getCount1(){
	return count1;
}

int Encoder::getCount2(){
	return count2;
}
