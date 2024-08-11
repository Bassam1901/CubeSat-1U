#include <Wire.h>
#include "BMP_lib.h"
extern long temp,pressure,Altitude;

short AC1=0;
short AC2=2;
short AC3=0;
unsigned short AC4=0;
unsigned short AC5=0;
unsigned short AC6=0;
short b1=0;
short B2=0;
short MB=0;
short MC=0;
short MD=0;

char oss=0;

void read_calib(){

  Wire.beginTransmission(0x77);
  Wire.write(0xAA);
  Wire.endTransmission();

  Wire.requestFrom(0x77,22);

  if (Wire.available()==22) {
    AC1=read2bytes();
    AC2=read2bytes();
    AC3=read2bytes();
    AC4=read2bytes();
    AC5=read2bytes();
    AC6=read2bytes();
    b1=read2bytes();
    B2=read2bytes();
    MB=read2bytes();
    MC=read2bytes();
    MD=read2bytes();
  }



  //Serial.println(AC1); Serial.println(AC2); Serial.println(AC3); Serial.println(AC4); Serial.println(AC5); Serial.println(AC6);
  //Serial.println(b1);  Serial.println(B2);  Serial.println(MB);  Serial.println(MC); Serial.println(MD);
}


void read_Temp_press_Alt(long* Temp,long* press,long* Altitude)
{
  //read UT//
  Wire.beginTransmission(0x77);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();

  delayMicroseconds(4500);

  Wire.beginTransmission(0x77);
  Wire.write(0xF6);
  Wire.write(0x2E);
  Wire.endTransmission();

  
  Wire.requestFrom(0x77,2);
  long ut=0;
   if (Wire.available()==2){
    ut=read2bytes();
   }

   //calculate Temperature
   long x1= (ut-AC6)*AC5/pow(2,15);
   long x2= MC*pow(2,11)/(x1 + MD);
   //long B= x1+x2
   *Temp=(x1+x2+8)/pow(2,4)/10;


   //Read UP
    
Wire.beginTransmission(0x77);
  Wire.write(0xF4);
  Wire.write(0x34+(oss<<6));
  Wire.endTransmission();
 
 switch (oss){
   case 0: 
    delayMicroseconds(4500);
    break;
   case 1:
    delayMicroseconds(7500);
    break;
   case 2:
    delayMicroseconds(13500);
    break;
   case 3:
    delay(25);
    delayMicroseconds(500);
    break; 

     
 }

 Wire.beginTransmission(0x77);
  Wire.write(0xF6);
  Wire.endTransmission();

  
  Wire.requestFrom(0x77,3);
  long up=0;
   if (Wire.available()==3){
    up=(long)read2bytes();
    up= ((long)up<<8 | (long)Wire.read())>>(8-oss);
   }
 //calculate pressure
  long B6 = (x1+x2)-4000;
  x1=(B2*(B6*B6/pow(2,12))/pow(2,11));
  x2=AC2*B6/pow(2,11);
  long x3=x1+x2;
  long B3=(((AC1*4+x3)<<oss)+2)/4;
  x1=AC3*B6/pow(2,13);
  x2=(b1*(B6*B6/pow(2,12))/pow(2,16));
  x3=((x1+x2)+2)/pow(2,2);
  unsigned long B4=AC4*(unsigned long)(x3+32768)/pow(2,15);
  unsigned long B7=((unsigned long)up-B3)*(50000>>oss);
  long p=0;
  if(B7<0x80000000){p=(B7*2)/B4;}
  else {p=B7/B4*2;}
  x1=(p/pow(2,8))*(p/pow(2,8));
  x1=(x1*3038)/pow(2,16);
  x2=(-7357*p)/pow(2,16);
  *press=p+(x1+x2+3791)/pow(2,4);


  //Calculate Altitude

*Altitude= 44330*(1-pow((*press/101325),(1/5.255)))-44330+9;

}







