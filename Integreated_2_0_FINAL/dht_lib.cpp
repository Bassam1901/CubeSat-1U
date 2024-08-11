#include "dht.h"
int temp_int,temp_deci,rh_int,rh_deci;

void init_dht(char pin) {
  pinMode(pin,INPUT);
  digitalWrite(pin,HIGH);

}
char read_dht (char pin,float* temp,float* rh)
{
  char data[40]={};
  //start signal
  pinMode(pin,OUTPUT);
  digitalWrite(pin, LOW);
  delay(18);
   digitalWrite(pin, HIGH);
    pinMode(pin,INPUT);
   //wait for response 40us
   delayMicroseconds(40);
   //checkresponce 
   if(digitalRead(pin)==HIGH){
     Serial.print("no response");

      return 1;
   }
   while (digitalRead(pin)==LOW);
   while (digitalRead(pin)==HIGH);
   //read data
   for (int i=0 ; i < 40 ;i++ ){
     while (digitalRead(pin)==LOW);
     delayMicroseconds(30);

     if (digitalRead(pin)==HIGH){ 
       data[i]=1;
       while (digitalRead(pin)==HIGH);
     }
   }
   int rh_int =0;
   for( int i=0; i<8 ;i++){
     rh_int = rh_int << 1 ;
     if(data[i]==1) rh_int |= 1;   
   }
int rh_deci =0;
   for( int i= 8; i<16 ;i++){
     rh_deci=rh_deci<<1;
     if(data[i]==1) rh_deci |= 1;   
   }
   int temp_int =0;
   for( int i= 16; i<24 ;i++){
     temp_int=temp_int<<1;
     if(data[i]==1) temp_int |= 1;   
   }
   int temp_deci =0;
   for( int i= 24 ; i<32 ;i++){
     temp_deci=temp_deci<<1;
     if(data[i]==1) temp_deci |= 1;   
   }
    int sum_check =0;
   for( int i=32 ; i < 40 ;i++){
    sum_check = sum_check << 1;
     if(data[i]==1) sum_check |= 1;   
   }
    
    Serial.print("513:");Serial.print(temp_int);Serial.print(".");Serial.print(temp_deci);
    Serial.print(",");Serial.print(rh_int);Serial.print(".");Serial.println(rh_deci);


}