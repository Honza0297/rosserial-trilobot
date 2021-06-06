 /************************************************ */
 /*  Educational tutorial for Arduino in robotics  */
 /*  Vyukovy Tutorial pro pouziti Arduina v robotice*/
 /*  File: I2C.h                                   */
 /*  Author: Jan Beran                             */
 /*  Date: March 2020                              */
 /*                                                */
 /* This file is a part of author´s bachelor thesis*/
 /*                                                */
 /**************************************************/

#ifndef _I2C_H
#define _I2C_H 1

#include <Arduino.h>
#include <Wire.h>

/** @brief Funkce zjednodusujici zapis do registru pres I2C */
void reg_write(byte slave_addr, byte reg, byte value);

#endif