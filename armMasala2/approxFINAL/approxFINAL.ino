#include "armMasala2.h"
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Servo.h>
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 38);

int s[] = {1600, 1460, 1740};
Servo yaw, pitch, roll;      

int yawstep = 3, pitchstep = 3, rollstep = 3; 

unsigned int localPort = 23909;
unsigned char data[UDP_TX_PACKET_MAX_SIZE];

int bits1[] = {0, 0, 0, 0, 0, 0, 0, 0};
int bits2[] = {0, 0, 0, 0, 0, 0, 0, 0};

int dat,mat;

EthernetUDP Udp;

int linearActuator1[4]={26,28,7,A5};                 //pin1,pin2,pwm,potPin {22,24,6,A3}
int linearActuator2[4]={22,24,6,A3};                 
int turntable[4]={1,127,3,A2};                       //l sVaule ,uValue ,Serialno,pot
int gripper[4]={129,255,3,0};
int TEMP=0,TEMP1=0;
armMasala2 arm;

//void display();
void initial();
void ypr();
bool getBit(int n, int pos);
void updateBits(int val1,int val2);
void process(int input1,int input2);
void setup () 
{
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);
  Serial.begin(9600);

  arm.boot(linearActuator1,linearActuator2,turntable,gripper);
  arm.LA1.velocity=80;
  arm.LA2.velocity=80;
  arm.GRP.velocity=50;
  arm.TT.velocity=30;
  Serial3.begin(9600);
  //yaw.attach(5);
  pitch.attach(4);//5 
  roll.attach(5);// 2

}
char c;
void loop ()
{
  int packetSize = Udp.parsePacket();
  if (packetSize) 
  {
    memset(data, 0, 24);
    Udp.read(data, UDP_TX_PACKET_MAX_SIZE);    
    dat = data[0];
    mat = data[1];
    process(dat,mat);    
  }
}

void updateBits(int val1,int val2) 
{
  for (int i = 0; i < 8; i++)
    {
    bits1[i] = getBit(val1, i);
    bits2[i] = getBit(val2, i);
    }
}


bool getBit(int n, int pos) 
{
  return (n >> pos) & 1;
}

void display()
{
  for (int i = 0; i < 8; i++) 
  {
    Serial.print(bits1[i]);
    Serial.print(" ");
  }
  Serial.print("             ");
  for (int i = 0; i < 8; i++) 
  {
    Serial.print(bits2[i]);
    Serial.print(" ");
  }
  
  Serial.println();
}

void process(int input1,int input2)
{
    updateBits(input1,input2);  
    display();
    if(!bits2[7])
    {
    arm.LA1.velocity=100;
    arm.LA2.velocity=100;
    arm.TT.velocity=30;
    arm.GRP.velocity=50;
    
    arm.LA1.run(bits1[0]&&!(bits2[0]||bits2[1]||bits2[2]),bits1[1]&&!(bits2[0]||bits2[1]||bits2[2]));    
    arm.LA2.run(bits1[4]&&!(bits2[0]||bits2[1]||bits2[2]),bits1[5]&&!(bits2[0]||bits2[1]||bits2[2]));
    arm.TT.run(bits1[2]&&!(bits2[0]||bits2[1]||bits2[2]),bits1[3]&&!(bits2[0]||bits2[1]||bits2[2]));
    arm.GRP.run(bits2[3],bits2[4]);

    arm.LA1.velocity=0;
    arm.LA2.velocity=0;
    }
    
    else
    {
     arm.gotoSemiDir((bits1[1]-bits1[0]),-1*(bits1[4]-bits1[5]),(bits1[2]-bits1[3]),80);  
    }
     if((bits1[6])||(TEMP==1))
      { 
      if(bits2[7]==0)  
      {TEMP=1; }  
      else              
      {TEMP=0; arm.LA1.run(0,0); arm.LA2.run(0,0); arm.TT.run(0,0);}
      arm.gotoPot(917,903,0,80);//position1
      Serial.println("RUNNING 1 ");
      }
      else if((bits1[7])||(TEMP1==1))
      {
      if(bits2[7]==0)  
      {  TEMP1=1;  }
      else              
      {TEMP1=0;  arm.LA1.run(0,0); arm.LA2.run(0,0); arm.TT.run(0,0);}
       arm.gotoPot(774,762,0,80);//position2
       Serial.println("RUNNING 2 ");
      }
    ypr2(); 
}

int safeservo_yaw(int x) {
  if (x > 1700) return 1700;
  else if (x < 1400) return 1400;
  else return x;
}

int safeservo_pitch(int x) {
  if (x > 2100) return 2100;
  else if (x < 1440) return 1440;
  else return x;
}

int safeservo_roll(int x) {
  if (x > 2400) return 2400;
  else if (x < 800) return 800;
  else return x;
}
void ypr2()
{
    s[0] += yawstep * ((bits1[4] - bits1[5])*bits2[0]);    s[0] = safeservo_yaw(s[0]);
    s[1] += pitchstep * ((bits1[0] - bits1[1])*bits2[1]);  s[1] = safeservo_pitch(s[1]);
    s[2] += rollstep * ((bits1[0] - bits1[1])*bits2[2]);   s[2] = safeservo_roll(s[2]);
    yaw.writeMicroseconds(s[0]);
    pitch.writeMicroseconds(s[1]);
    roll.writeMicroseconds(s[2]);
}
void initial()
{ s[0] = 1600; s[1] = 1460 ; s[2] = 1740;
  yaw.writeMicroseconds(s[0]);
  pitch.writeMicroseconds(s[1]);
  roll.writeMicroseconds(s[2]);
}
