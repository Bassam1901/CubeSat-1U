#include "Arduino.h"
#include <Wire.h>



#ifndef BMP_H
#define BMP_H

#define read2bytes() ((long)Wire.read()<<8 |(long) Wire.read());
extern char oss;
void read_calib();


void read_Temp_press_Alt(long* Temp,long* press,long* Altitude);


#endif