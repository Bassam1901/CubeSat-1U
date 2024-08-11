const int buzzerPin = 9;

//RF
#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>
#define pot A0
#define CE 7
#define CSN 6
bool check;
RF24 CUBE(CE, CSN);
const byte adresses[][6]={"HMH12", "HMH13"};

struct data {
  long T,P,A;
  int T_i,RH_i;
  float Phai,Theta,Psi,Accel_X,Accel_Y,Accel_Z;
  char ID[6],ALT[7],LONG[9],LAT[9];
  String crc,crc2;

}BMP_DHT,MPU,GPS;

//SD
#include <SPI.h>
#include <SD.h>
#define SD_CS 10
File readFile;

//Voltage Sensor
#include "Voltage_Sensor.h"


//BMP
#include "BMP_lib.h"
long Altitude = 0;
long Temp = 0;
long pressure = 0;

//DHT
#include "dht.h"
float T = 0;
float Rh = 0;
float *rh;
float *temp;
extern int temp_int,temp_deci,rh_int,rh_deci;


//MPU
#include <Wire.h>
#include <math.h>
#include "MPU.h"
#define MPU_ADDRESS 0x68
#define SMPRT_DIV 0X19
#define CONFIG 0x1A
#define PWR_MGMNT 0x6B
#define ACCEL_XOUTM 0x3B
#define GYRO_XOUTM 0x43
#define MPI 3.14159265
#define L 0.1


float accel_x, accel_y, accel_z, temp2, p, q, r;
float accel_x_bias, accel_y_bias, accel_z_bias, temp_bias, p_bias = 0, q_bias = 0, r_bias = 0;
float phai_dot, theta_dot, psi_dot;
float phai_gyro = 0, theta_gyro = 0, psi_gyro = 0;
float phai_accel = 0, theta_accel = 0;
float phai = 0, theta = 0, psi = 0;

//GPS
////$GPGGA,033410.000,2232.1745,N,11401.1920,E,1,07,1.1,107.14,M,0.00,M,,*64

char mid[6];
char utc[11];
char lat[10];
char n_s[2];
char lng[11];
char e_w[2];
char posfix[2];
char sat[3];
char hdop[4];
char alt[7];

//CRC
#include "CRC.h"


void setup() {
  Serial.begin(9600);

  //RF_SEND
   CUBE.begin();
   CUBE.setPALevel(RF24_PA_MAX);
   CUBE.enableDynamicPayloads();
   CUBE.openWritingPipe(adresses[1]);
   CUBE.openReadingPipe(1, adresses[0]);

  //SD
  if(!SD.begin(SD_CS)){
    Serial.println("CARD FAILED,OR NOT PRESENT!!");

  }

  char filename[]="READ00.txt";
  for(uint8_t i =0;i<100;i++){
    filename[4]=i/10+'0';
    filename[5]=i%10+'0';
    if (!SD.exists(filename)){
      readFile=SD.open(filename,FILE_WRITE);
      break;
      }
  }

  //BMP
  Wire.begin();
  oss = 0;

  //DHT
  init_dht(5);

  //MPU
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(SMPRT_DIV);
  Wire.write(31);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(PWR_MGMNT);
  Wire.write(0x01);
  Wire.endTransmission(true);
  for (int i; i = 0; i < 100) {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(ACCEL_XOUTM);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU_ADDRESS, 14);
    if (Wire.available() > 13) {
      accel_x_bias += (Wire.read() << 8 | Wire.read()) / 16384.0;
      accel_y_bias += (Wire.read() << 8 | Wire.read()) / 16384.0;
      accel_z_bias += (Wire.read() << 8 | Wire.read()) / 16384.0;
      temp_bias += (Wire.read() << 8 | Wire.read()) / 340.0 + 35.53;
      p_bias += (Wire.read() << 8 | Wire.read()) / 131.0 * MPI / 180.0;
      q_bias += (Wire.read() << 8 | Wire.read()) / 131.0 * MPI / 180.0;
      r_bias += (Wire.read() << 8 | Wire.read()) / 131.0 * MPI / 180.0;
    }
  }
  accel_x_bias /= 100;
  accel_y_bias /= 100;
  accel_z_bias /= 100;
  temp_bias /= 100;
  p_bias /= 100;
  q_bias /= 100;
  r_bias /= 100;
  accel_z_bias -= 1;
}

void loop() {
  Voltage();

  //BMP
  read_calib();
  read_Temp_press_Alt(&Temp, &pressure, &Altitude);
  String T_P_A=String(Temp,DEC)+","+String(pressure,DEC)+","+String(Altitude,DEC);

  Serial.print("512:");
  Serial.print(Temp);
  Serial.print(",");
  Serial.print(pressure);
  Serial.print(",");
  Serial.println(Altitude);
  CRC_CALC(T_P_A);


  //DHT
  read_dht(5, &T, &Rh);
  float *rh = &Rh;
  float *temp = &T;
  String T_RH=String(temp_int,DEC)+"."+String(temp_deci,DEC)+","+String(rh_int,DEC)+"."+String(rh_deci);
  CRC_CALC(T_RH);

  //MPU
  long T1 = micros();
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(ACCEL_XOUTM);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDRESS, 14);
  if (Wire.available() > 13) {
    accel_x = (Wire.read() << 8 | Wire.read()) / 16384.0 - accel_x_bias;
    accel_y = (Wire.read() << 8 | Wire.read()) / 16384.0 - accel_y_bias;
    accel_z = (Wire.read() << 8 | Wire.read()) / 16384.0 - accel_z_bias;
    temp2 = (Wire.read() << 8 | Wire.read()) / 340.0 + 35.53 - temp_bias;
    p = (Wire.read() << 8 | Wire.read()) / 131.0 * MPI / 180.0 - p_bias;
    q = (Wire.read() << 8 | Wire.read()) / 131.0 * MPI / 180.0 - q_bias;
    r = (Wire.read() << 8 | Wire.read()) / 131.0 * MPI / 180.0 - r_bias;
  }

  phai_dot = p + q * sin(phai) * tan(theta) + r * cos(phai) * tan(theta);
  theta_dot = q * cos(phai) - r * sin(phai);
  psi_dot = q * sin(phai) / cos(theta) + r * cos(phai) / cos(theta);

  phai_gyro = phai + phai_dot * 0.004;
  theta_gyro = theta + theta_dot * 0.004;
  psi_gyro = psi + psi_dot * 0.004;

  phai_accel = atan(accel_y / accel_z);
  if (abs(accel_x) > 1) {
    theta_accel = asin(1);
  } else {
    theta_accel = asin(accel_x / 1);
  }
  phai = phai_gyro + L * (phai_accel - phai_gyro);
  theta = theta_gyro + L * (theta_accel - theta_gyro);
  psi = psi_gyro;

  Serial.print("514:");
  Serial.print(phai * 180.0 / MPI);
  Serial.print(',');
  Serial.print(theta * 180.0 / MPI);
  Serial.print(',');
  Serial.print(psi * 180.0 / MPI);
  Serial.print(',');
  Serial.print(accel_x);
  Serial.print(',');
  Serial.print(accel_y);
  Serial.print(',');
  Serial.println(accel_z);
  String M_P_U=String(phai * 180.0 / MPI,DEC)+","+String(theta * 180.0 / MPI,DEC)+","+String(psi * 180.0 / MPI,DEC)+","+String(accel_x,DEC)+","+String(accel_y,DEC)+","+String(accel_z,DEC);
  CRC_CALC(M_P_U);

  delayMicroseconds(4000 - (micros() - T1));

  //GPS
  // Serial.read() != '$'
  unsigned int x = millis();
  String G_P_S=String(mid)+","+String(alt)+","+String(lng)+","+String(lat);
  
  while (millis() - x < 2000) {
    if (Serial.read() == '$') {
      Serial.readBytesUntil(',', mid, 6);
      //Check message ID
      if (mid[2] == 'G' && mid[3] == 'G' && mid[4] == 'A') {
        Serial.readBytesUntil(',', utc, 11);
        Serial.readBytesUntil(',', lat, 10);
        Serial.readBytesUntil(',', n_s, 2);
        Serial.readBytesUntil(',', lng, 11);
        Serial.readBytesUntil(',', e_w, 2);
        Serial.readBytesUntil(',', posfix, 2);
        Serial.readBytesUntil(',', sat, 3);
        Serial.readBytesUntil(',', hdop, 4);
        Serial.readBytesUntil(',', alt, 7);

        mid[5] = '\0';
        utc[10] = '\0';
        lat[9] = '\0';
        n_s[1] = '\0';
        lng[10] = '\0';
        e_w[1] = '\0';
        posfix[1] = '\0';
        sat[2] = '\0';
        hdop[3] = '\0';
        alt[6] = '\0';

        Serial.print("515:");
        Serial.print(",");
        Serial.print(mid);
        Serial.print(",");                
        Serial.print(alt);
        Serial.print(",");
        Serial.print(lng);
        Serial.print(",");
        Serial.println(lat);
        CRC_CALC(G_P_S);


        break;
      }
    }
  }

  //SD Write

  readFile.print(millis());
  readFile.print("|");
  readFile.print("512:");
  readFile.print(Temp);
  readFile.print(",");
  readFile.print(pressure);
  readFile.print(",");
  readFile.println(Altitude);

  readFile.print(millis());
  readFile.print("|");
  readFile.print("513:");
  readFile.print(temp_int);
  readFile.print(".");
  readFile.print(temp_deci);
  readFile.print(",");
  readFile.print(rh_int);
  readFile.print(".");
  readFile.println(rh_deci);

  readFile.print(millis());
  readFile.print("|");
  readFile.print("514:");
  readFile.print(phai*180.0/MPI);
  readFile.print(",");
  readFile.print(theta*180.0/MPI);
  readFile.print(",");
  readFile.print(psi*180.0/MPI);
  readFile.print(",");
  readFile.print(accel_x);
  readFile.print(",");
  readFile.print(accel_y);
  readFile.print(",");
  readFile.println(accel_z);

    readFile.print(millis());
    readFile.print("|");
    readFile.print("515:");
    readFile.print(mid);
    readFile.print(",");
    readFile.print(alt);
    readFile.print(",");
    readFile.print(lng);
    readFile.print(",");
    readFile.print(lat);



  readFile.flush();

  //RF_SEND
  CUBE.startListening();
  delay(100);

  //RF_SEND -->SAVING DATA IN STRUCTS
  BMP_DHT.T=Temp;
  BMP_DHT.P=pressure;
  BMP_DHT.A=Altitude;    //FIRST STRUCT
  BMP_DHT.T_i=temp_int;
  BMP_DHT.RH_i=rh_int;
  BMP_DHT.crc=CRC_CALC(T_P_A);
  BMP_DHT.crc2=CRC_CALC(T_P_A);

  if(CUBE.available() > 0) {
     CUBE.write(&BMP_DHT, sizeof(data));
  }


  MPU.Phai=phai*180/MPI;
  MPU.Theta=theta*180/MPI;
  MPU.Psi=psi*180/MPI;
  MPU.Accel_X=accel_x; //SECOND STRUCT
  MPU.Accel_Y=accel_y;
  MPU.Accel_Z=accel_z;
  MPU.crc=CRC_CALC(M_P_U);

    if(CUBE.available() > 0) {
    CUBE.write(&MPU, sizeof(data));    
  }

  GPS.ID[6]=mid[6];
  GPS.ALT[7]=alt[7]; // THIRD STRUCT
  GPS.LONG[9]=lng[10];
  GPS.LAT[9]=lat[11];
  GPS.crc=CRC_CALC(G_P_S);


  if(CUBE.available() > 0) {
    CUBE.write(&GPS, sizeof(data));    
  }
  CUBE.stopListening();

//BUZZER SEND COMMAND
  // CUBE.startListening();

  // if(CUBE.available() > 0) {
  //   CUBE.read(&sym,sizeof((data2)));
  //   pinMode(buzzerPin, OUTPUT);
  //   digitalWrite(buzzerPin, 127);
  //   delay(500);
  //   digitalWrite(buzzerPin,0);
  //   delay(500);
  // }
  // CUBE.stopListening();



  
}
