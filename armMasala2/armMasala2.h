#ifndef __ARMMASALA2_H__
#define __ARMMASALA2_H__
#include "Arduino.h"

#define FACTOR 0.017453                                         //degree to radian conversion factor
//#define PI 3.14                                                 //Value of Pi


class motor
{
  public:
    int pin[4];                                                 //pin1,pin2,pwm,potPin
    int potVal;                                                 //pot value;
    int velocity=50;                                               //pwn value for particular motor
    void setPins(int temp[]);                                        //initialises the pins
    void run(int p1,int p2);                                    //runs motor forward(1,0) or backward(0,1) of pause(0,0) 
    int getPotVal();                                            //Returns potVal
};

class saberMotor
{
  public:
    int saberParameters[4];                                     //lower serialVaule ,upperSerialValue ,Serial no ,pot
    int potVal;                                                 //holds the pot value
    int ifRunningForFirstTime=1;                                //holds value 1 if running for the first time
    double acc=0.05;                                            //acceleration up value
    double current=0;                                           //current state varies from -1 to +1
    double velocity=50;                                         //varies from 0 to 80
    int currentSpeed=0;
    void setPins(int temp[]);
    void run(int p1,int p2);
    int getPotVal();
};

class armMasala2
{
public:
  motor LA1;                                                    //LA1-Bottom Actuator
  motor LA2;                                                    //LA2-Top Actuator
  saberMotor TT;                                                     //Turn Table
  saberMotor GRP;
  
double L[10]={0,500,332,367,55,95,306,77,375,20};                 //constant link lengths NEW.
double TP[6]={50,65,60,46.946*FACTOR,71.79*FACTOR,61.264*FACTOR};//constant triangle  Parameters
double FA[7]={atan(L[9]/L[6]),acos(L[3]/L[8]),21.66*FACTOR,45*FACTOR,46.946*FACTOR,71.79*FACTOR,61.264*FACTOR};
double TL[2]={-90*0.017453,90*0.017453};                        //min and max angle limits for turn table 
double TLinDegrees[2]={-90,90};
double TTP[2]={750,275};//750,275                               //min and max pot value for turn table
double AL[4]={255,340,352,445};                                  //actuator length Limits
double AP[4]={960,34,999,36}; 
double coordinates3D[3];                                        //current 3D coordinates...

  void boot(int[],int[],int[],int[]);                                 //initialises all pins by calling setPins()
  void gotoPot(double,double,double,double);
  void gotoSemiDir(double,double,double,double);
 };

#endif
