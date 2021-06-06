 /************************************************ */
 /*  Educational tutorial for Arduino in robotics  */
 /*  Vyukovy Tutorial pro pouziti Arduina v robotice*/
 /*  File: I2C.cpp                                 */
 /*  Author: Jan Beran                             */
 /*  Date: March 2020                              */
 /*                                                */
 /* This file is a part of author´s bachelor thesis*/
 /*                                                */
 /**************************************************/

#include "I2C.h"
/**
 * @brief Funkce pro zejdnoduseni zapisu do registru pres I2C sbernici.
 * */
void reg_write(byte slave_addr, byte reg, byte value)
{
  Wire.beginTransmission(slave_addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}