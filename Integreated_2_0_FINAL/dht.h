#include "Arduino.h"

#ifndef DHT_H
#define DHT_H

extern int temp_int,temp_deci,rh_int,rh_deci;

void init_dht(char pin);
char read_dht (char pin,float* temp,float* rh);



#endif