#ifndef __ARMMASALA_H__
#define __ARMMASALA_H__
#include "Arduino.h"

#define FACTOR 0.017453                                         //degree to radian conversion factor
#define PI 3.14                                                 //Value of Pi

double L[]={0,500,332,367,55,95,306,77,375,20};                 //constant link lengths NEW.
double TP[]={50,65,60,46.946*FACTOR,71.79*FACTOR,61.264*FACTOR};//constant triangle  Parameters
double FA[]={atan(L[9]/L[6]),acos(L[3]/L[8]),21.66*FACTOR,45*FACTOR,46.946*FACTOR,71.79*FACTOR,61.264*FACTOR};
double TL[2]={-90*0.017453,90*0.017453};                        //min and max angle limits for turn table 
double TLinDegrees[2]={-90,90};
double TTP[2]={750,275};//750,275                               //min and max pot value for turn table
double AL[]={255,340,352,445};                                  //actuator length Limits
double AP[]={960,34,999,36}; 
double coordinates3D[3];                                        //current 3D coordinates...

class motor
{
  public:
    int pin[4];                                                 //pin1,pin2,pwm,potPin
    int potVal;                                                 //pot value;
    int velocity;                                               //pwn value for particular motor
    void setPins(int[]);                                        //initialises the pins
    void run(int p1,int p2);                                    //runs motor forward(1,0) or backward(0,1) of pause(0,0) 
    void serialRun(int p1,int p2);
    int getPotVal();                                            //Returns potVal
};
class armMasala
{
public:
  motor LA1;                                                    //LA1-Bottom Actuator
  motor LA2;                                                    //LA2-Top Actuator
  motor TT;                                                     //Turn Table

  void boot(int[],int[],int[]);                                 //initialises all pins by calling setPins()
  void gotoPot(double,double,double,double);
  void gotoSemiDir(double,double,double,double);
 };

#endif