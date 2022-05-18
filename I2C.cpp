/************************************************ 
 /*  Educational tutorial for Arduino in robotics  
/*    AND
/*  Docking Station for Automatic Charging of Batteries of Robot 
/*  File: I2C.cpp                                 
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