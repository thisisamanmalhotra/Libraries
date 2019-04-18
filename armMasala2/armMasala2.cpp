#include "armMasala2.h"

void motor::setPins(int temp[4])
{ for(int i=0;i<4;i++)
    pin[i]=temp[i];
  for(int i=0;i<3;i++)
    pinMode(pin[i],OUTPUT);
    pinMode(pin[3],INPUT);
  velocity=0;
}

void motor::run(int p1,int p2)                                  //1,0 -Forward 0,1-Backward   0,0-Pause
{
  analogWrite(pin[2],velocity);
  digitalWrite(pin[0],p1);
  digitalWrite(pin[1],p2);  
}

int motor::getPotVal()
{ 
  potVal=analogRead(pin[3]);
  return potVal;
}
void saberMotor::setPins(int temp[])
{for(int i=0;i<4;i++)
 saberParameters[i]=temp[i];
 currentSpeed=0;
 if(temp[3]!=0)
  pinMode(saberParameters[3],INPUT);
}
void saberMotor::run(int p1,int p2)
{
  int serialValue;
  if(ifRunningForFirstTime==1)
    { current=0; ifRunningForFirstTime=0;}
  int state=p1-p2;
  
   current = state > current ? current+acc : current;
   current = state < current ? current-acc : current;
   current = current >1 ? 1 : current;
   current = current <-1? -1: current;
   currentSpeed = current * velocity;
  
  serialValue=map(currentSpeed,-80,80,saberParameters[0],saberParameters[1]);
  
  if(abs(currentSpeed)<=10)
  {
    if(saberParameters[0]=1)   serialValue=64;
    else                       serialValue=192;
  }

  switch(saberParameters[2])
  {
    case 1: Serial1.write(serialValue);
        break;
    case 2: Serial2.write(serialValue);
        break;
    case 3: Serial3.write(serialValue);
  }
}
int saberMotor::getPotVal()
{ 
  potVal=analogRead(saberParameters[4]);
  return potVal;
}
void armMasala2::boot(int m1Pin[4],int m2Pin[4],int m3Pin[4],int m4Pin[4])
{
  LA1.setPins(m1Pin);
  LA2.setPins(m2Pin);
  TT.setPins(m3Pin);
  GRP.setPins(m4Pin); 
}

void armMasala2::gotoPot(double la1,double la2,double tt,double vel)
{
while( (fabs(la1-LA1.getPotVal())>20 || fabs(la2-LA2.getPotVal())>20 ) )// ||fabs(tt-TT.getPotVal())>20
  {
  double  del1=fabs(la1-LA1.getPotVal());
   double del2=fabs(la2-LA2.getPotVal());
    double  del3=fabs(tt-TT.getPotVal());
    double tempvel1,tempvel2;
    if(del1>del2)
    { 
      LA1.velocity=vel;
      tempvel1= vel*(del2/(del1)); LA2.velocity=tempvel1;
     // tempvel2= vel*(del3/del1);    TT.velocity=tempvel2;
    }
    else
    {
      LA2.velocity=vel;
      
      tempvel1=vel*(del1/(del2)); LA1.velocity=tempvel1;
     // tempvel2=vel*(del3/del2);   TT.velocity=tempvel2;
    }

    la1>LA1.getPotVal()?LA1.run(1,0):LA1.run(0,1);
    la2>LA2.getPotVal()?LA2.run(1,0):LA2.run(0,1);
    //tt>TT.getPotVal()?TT.run(1,0):TT.run(0,1);
  }
  TT.run(0,0);
  LA2.run(0,0);
  LA1.run(0,0);
}
void armMasala2::gotoSemiDir(double d1,double d2,double d3,double vel)
{
 double dXdt=d1*vel;
 double dYdt=d2*vel;
 double dZdt=d3*vel;
  
  double BETA=map(LA2.getPotVal(),AP[2],AP[3],AL[2],AL[3]);                   //length of the lower actuator
  double ALPHA=map(LA1.getPotVal(),AP[0],AP[1],AL[0],AL[1]);                  //length of the top actuator
  double THETA=acos((pow(L[6],2)+pow(L[7],2)-pow(ALPHA,2))/ (2*L[6]*L[7]))+FA[0]-FA[2];

  double lengths=pow((pow(TP[1],2)+pow(L[8],2)-2*TP[1]*L[8]*cos(PI+TP[5]-THETA-FA[3])),0.5);
  double H=asin((L[8]*sin(PI+TP[5]-THETA-FA[3]))/(lengths));
  double Q=acos((pow(L[5],2)+pow(lengths,2)-pow(BETA,2))/(2*L[5]*lengths))-H;
  double FI=Q-PI+TP[3]+THETA+TP[4]+FA[3];
 
  double X=(L[1]*cos(THETA))+(L[2]*cos(THETA-FI));                                    //in cylindrical system...
  double Y=(L[1]*sin(THETA))+(L[2]*sin(THETA-FI));                                    //in cylindrical system...
 
  double LAMBDA=pow((pow(TP[1],2)+pow(L[5],2)-2*(TP[1])*(L[5])*cos(Q)),0.5);
  double DELTA=acos((pow(TP[1],2)+pow(LAMBDA,2)-pow(L[5],2))/(2*(TP[1])*(LAMBDA)));
  double GAMA=(PI-DELTA-FA[1]+THETA+FA[3]-FA[6]);

   
  double dFIdt=(-1*(((X*dXdt)+(Y*dYdt))/(L[1]*L[2]*sin(FI))));
  double dTHETAdt=( ( (L[2]*sin(THETA-FI)*dFIdt)-(dXdt) )/( (L[1]*sin(THETA))+(L[2]*sin(THETA-FI)) ) );
   
  double pre= ( (L[6]*L[7]*sin(FA[2]+THETA-FA[0]))/(pow(( (pow(L[6],2))+(pow(L[7],2))-(2*L[6]*L[7]*cos(FA[2]+THETA-FA[0]))),0.5)) );
  double dALPHAdt=pre*dTHETAdt;

  double dQdt=(dFIdt-dTHETAdt);
  double dLAMBDAdt=( (TP[1]*L[5]*sin(Q)*dQdt)/(LAMBDA));
  double dDELTAdt=( ((dLAMBDAdt)*((TP[1]*cos(DELTA))-LAMBDA))/(TP[1]*LAMBDA*sin(DELTA)) );
  double dGAMAdt=((dTHETAdt)-(dDELTAdt));
  double dBETAdt=( ( (LAMBDA*dLAMBDAdt)-((L[8])*((cos(GAMA)*dLAMBDAdt)-(LAMBDA*sin(GAMA)*dGAMAdt)) ))/(BETA) );
 

 double error=10;
 double error2=1;

 LA1.velocity=fabs(error*dALPHAdt);
 LA2.velocity=fabs(error*dBETAdt);
 //TT.velocity=fabs(error2*dZdt);
 
 if     (dALPHAdt>0)  LA1.run(1,0);
 else if(dALPHAdt<0)  LA1.run(0,1);
 else                 LA1.run(0,0);
 
 if     (dBETAdt>0)   LA2.run(1,0);
 else if(dBETAdt<0)   LA2.run(0,1);
 else                 LA2.run(0,0);

 if     (dZdt>0)      TT.run(1,0);
 else if(dZdt<0)      TT.run(0,1);
 else                 TT.run(0,0);
}
