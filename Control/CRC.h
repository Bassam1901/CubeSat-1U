#include "CRC16.h"


CRC16 crc(CRC16_MODBUS_POLYNOME,
          CRC16_MODBUS_INITIAL,
          CRC16_MODBUS_XOR_OUT,
          CRC16_MODBUS_REV_IN,
          CRC16_MODBUS_REV_OUT);

String CRC_CALC(String var){
  
  crc.restart();  
  for (int i = 0; i < var.length(); i++)
  {
    crc.add(var[i]);
  }
  Serial.println(crc.calc(), HEX);
}