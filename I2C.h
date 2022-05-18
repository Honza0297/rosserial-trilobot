 /************************************************ 
 /*  Educational tutorial for Arduino in robotics  
 /*    AND
 /*  Docking Station for Automatic Charging of Batteries of Robot 
  /*  File: I2C.h                                   
 /*  Author: Jan Beran                             
 /*  Date: March 2020                              
 /*                                                
/* This file is a part of author´s diploma thesis
 /*                                                
 /* Justification to include it in the diploma thesis: 
 /* This function is still in use for I2C operations. 
 /* This file WAS NOT CHANGED SINCE THE BACHELOR THESIS
 /* (excluding this header)
 /*                                                
 /**************************************************/

#ifndef _I2C_H
#define _I2C_H 1

#include <Arduino.h>
#include <Wire.h>

/** @brief Funkce zjednodusujici zapis do registru pres I2C */
void reg_write(byte slave_addr, byte reg, byte value);

#endif