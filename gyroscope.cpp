 /************************************************ 
 /*  Educational tutorial for Arduino in robotics  
 /*    AND
 /*  Docking Station for Automatic Charging of Batteries of Robot
 /*  File: gyroscope.cpp                           
 /*  Author: Jan Beran                             
 /*  Date: March 2020                              
 /*                                                
 /* This file is a part of author´s diploma thesis
 /*                                                
 /* Justification to include it in the diploma thesis: 
 /* minIMUv3 is mounted on current Trilobots as well 
 /* and once will be integrated. 
 /* By now, this file WAS NOT CHANGED SINCE THE BACHELOR THESIS
 /* (excluding this header)
 /**************************************************/

#include <Wire.h>
#include "gyroscope.h"
#include "I2C.h"

Gyroscope::Gyroscope(byte address)
{
  Wire.begin();
  this->gyro_address = address;
  this->BIAS = {300,-1600,100};
  reg_write(this->gyro_address, LOW_ODR, 0x00);
  reg_write(this->gyro_address, CTRL4, 0x00);
  reg_write(this->gyro_address, CTRL1, 0x6F);
  delay(10);
  /*Pokud jsou hodntoy spatne, zkuste zakomentovat radek nize*/
  this->get_bias();
}

void Gyroscope::get_bias(int num_of_measurements)
{
  this->BIAS = {0,0,0};
  vector<int16_t> vals = {0,0,0};
  for(int i = 0; i < num_of_measurements; i++)
  {
   vector<int16_t> temp = get_raw_data();
    vals.x += temp.x;
    vals.y += temp.y;
    vals.z += temp.z;
    delay(5);
  }

  this->BIAS.x = vals.x/num_of_measurements;
  this->BIAS.y = vals.y/num_of_measurements;
  this->BIAS.z = vals.z/num_of_measurements;  
}

vector<float> Gyroscope::get_angular_velocity()
{
  vector<int16_t> values = this->get_raw_data();
  vector<float> return_vec;
  return_vec.x = (values.x-this->BIAS.x)*GYRO_CONVERSION_RATE;
  return_vec.y = (values.y-this->BIAS.y)*GYRO_CONVERSION_RATE;
  return_vec.z = (values.z-this->BIAS.z)*GYRO_CONVERSION_RATE;
  
  /*In degrees per second*/
  return return_vec;
}

vector<int16_t> Gyroscope::get_raw_data()
{
  vector<int16_t> return_vec;

  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(OUT_X_L | (1 << 7)); // horni bit 1 a pak 7-bit adresa registru 0x28
  Wire.endTransmission();

  Wire.requestFrom(GYRO_ADDRESS, 6);
  while(Wire.available() < 6);
  
  int8_t xl = Wire.read();
  int8_t xh = Wire.read();
  
  int8_t yl = Wire.read();
  int8_t yh = Wire.read();
  
  int8_t zl = Wire.read();
  int8_t zh = Wire.read();
   

  return_vec.x = (int16_t)((xh << 8) + xl);
  return_vec.y = (int16_t)((yh << 8) + yl);
  return_vec.z = (int16_t)((zh << 8) + zl);
  return return_vec;
}

byte Gyroscope::get_temperature()
{
  int temp_bias = 36;
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(OUT_TEMP | (1 << 7)); // horni bit 1 a pak 7-bit adresa registru
  Wire.endTransmission();

  Wire.requestFrom(GYRO_ADDRESS, 1);
  while(Wire.available() < 1);
  
  byte temp = Wire.read();
  /* Vestaveny senzor teploty neni urceny k mereni okolni teploty prostredi. 
   * V kazdem pripade pro tento ucel MUZE byt pouzit, jelikoz Trilobot jinym senzorem teploty nedisponuje. 
   * Hodnota temp_bias byla ziskana pomoci experimentu a reverzniho inzenyrstvi, jelikoz tato funkcionalita
   * neni popsana v dokumentaci.*/
  return temp_bias-temp;
}

bool Gyroscope::check_shake(float treshold)
{
  static vector<float> old = {0,0,0};
  vector<float> shake = {0,0,0};
  vector<float> data = this->get_angular_velocity();
  
  shake.x = abs(old.x-data.x);
  shake.y = abs(old.y-data.y);
  shake.z = abs(old.z-data.z);
  
  old.x = data.x;
  old.y = data.y;
  old.z = data.z;
  Serial.println(shake.x);
  if(abs(shake.x) > treshold || abs(shake.y > treshold) || abs(shake.z > treshold))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool Gyroscope::detektor_otresu(float treshold)
{
  /* TODO ziskat dvoje data s urcitou casovou prodlevou 
  - treba 20 ms */
  vector<float> first = {0,0,0};
  vector<float> second = {0,0,0};
  vector<float> shake = {0,0,0};
  
  /*Ziskame jejich rozdil */
  shake.x = abs(first.x-second.x);
  shake.y = abs(first.y-second.y);
  shake.z = abs(first.z-second.z);
  
  /* TODO pokud bude nektery z rozdilu v ose x,y,z vetsi nez 
  treshold - hranice "klidu", vratime true, jinak false */
 return false;
}