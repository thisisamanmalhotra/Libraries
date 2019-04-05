/*
 This programm controlls the arm without normalised controlls
 but have functions to take it to particular 3D coordinates
*/
#include "armMasala.h"
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Servo.h>

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 38);

int s[] = {1760, 1890, 1740};
Servo yaw, pitch, roll;      

int yawstep = 1, pitchstep = 1, rollstep = 1; 

unsigned int localPort = 23909;
unsigned char data[UDP_TX_PACKET_MAX_SIZE];

int bits1[] = {0, 0, 0, 0, 0, 0, 0, 0};
int bits2[] = {0, 0, 0, 0, 0, 0, 0, 0};

int dat,mat;

EthernetUDP Udp;

int linearActuator1[4]={45,43,12,A3};                 //pin1,pin2,pwm,potPin
int linearActuator2[4]={31,33,11,A5};
int turntable[4]={0,0,0,A2};//A2

armMasala arm;
void display();
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
//Serial1.begin(9600);
  arm.boot(linearActuator1,linearActuator2,turntable);
  yaw.attach(10);
  pitch.attach(5);//5 
  roll.attach(2);// 2
  initial();
  
}

void loop ()
{
  int packetSize = Udp.parsePacket();
  Serial.println(packetSize);
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
    arm.LA1.velocity=50;
    arm.LA2.velocity=50;

    arm.LA1.run(bits1[0],bits1[1]);    
    arm.LA2.run(bits1[2],bits1[3]);
    arm.TT.serialRun(bits1[4],bits1[5]);
    
    arm.LA1.velocity=0;
    arm.LA2.velocity=0;
   
    if(bits1[6]==1)
      arm.gotoPot(500,500,500,80);//position1
    if(bits1[7]==1)
      arm.gotoPot(300,800,500,80);//position2
    ypr();
}

int safeservo_yaw(int x) {

  if (x > 1900) return 1900;
  else if (x < 1600) return 1600;
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


void ypr() {

  if (bits2[1] && bits2[2] && bits2[3] && (!bits2[0])) {
    initial();
  }
  else if (bits2[3]) {
    s[0] += yawstep * (bits2[7] - bits2[6]);
    s[0] = safeservo_yaw(s[0]);
    yaw.writeMicroseconds(s[0]);
  }
  else if (bits2[2]) {
    s[1] += pitchstep * (bits2[5] - bits2[4]);
    s[1] = safeservo_pitch(s[1]);
    pitch.writeMicroseconds(s[1]);
  }
  else if (bits2[1]) {
    s[2] += rollstep * (bits2[7] - bits2[6]);
    s[2] = safeservo_roll(s[2]);
    roll.writeMicroseconds(s[2]);
  }
}
void initial()
{
  //  Initialize YPR
  //  Serial.println("Intializing YPR");
  s[0] = 1760; s[1] = 1890 ; s[2] = 1740;
  yaw.writeMicroseconds(s[0]);
  pitch.writeMicroseconds(s[1]);
  roll.writeMicroseconds(s[2]);
}

