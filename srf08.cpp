 /************************************************ 
 /*  Educational tutorial for Arduino in robotics  
 /*    AND
 /*  Docking Station for Automatic Charging of Batteries of Robot
 /*  File: srf08.cpp                               
 /*  Author: Jan Beran                             
 /*  Date: autumn 2019 and 2020-2022                            
 /*  Description: Gets sonar readings from all six sonars and sends the data to Raspberry Pi
 /*                                              
 /* This file is a part of author´s diploma thesis.
/* This file used parts of code from author's bachelor thesis 
 /*                                                
 /**************************************************/

#include <Arduino.h>
#include <Wire.h>
#include "srf08.h"
#include <std_msgs/Empty.h>




void Sonar_driver::update()
{
  if(this->measuring)
    {
       if( millis()-this->measure_start >= SRF08_MEASURE_TIME) // measure done
       {
         this->measuring = false;
         this->get_and_send_data();
       }
    }
  else // not measuring 
    {  
      if(millis()-this->last_updated >= SRF08_UPDATE_INTERVAL)
      {
        this->last_updated = millis();
        this->measuring = true;
        this->measure_start = millis();
        this->sonar->set_measurement();
      } 
    } 
 }

void Sonar_driver::get_and_send_data()
{
  sonar_data data = this->sonar->get_distances();

  this->msg.front = data.front;
  this->msg.front_right = data.front_right;
  this->msg.front_left = data.front_left;
  this->msg.back_right= data.back_right;
  this->msg.back_left = data.back_left;
  this->msg.back = data.back;

  this->pub.publish(&this->msg);
}



Sonar::Sonar()
{
  Wire.begin();
}


void Sonar::set_measurement(byte unit, uint8_t address)
{
  Wire.beginTransmission(address);
  Wire.write(REG_CMD);
  Wire.write(unit);
  Wire.endTransmission();
}

sonar_data Sonar::get_distances(byte unit)
{
  uint16_t data[6];  
  /* Ignoring first two registers*/
  for(int i = 0; i < NUM_OF_SONARS; i++)
  {
    Wire.beginTransmission(srf08_addresses[i]);
    Wire.write(0x02);                           
    Wire.endTransmission();

    Wire.requestFrom(srf08_addresses[i], 2); 
    while(Wire.available() < 0);
    byte high =  Wire.read();
    byte low =  Wire.read();
    data[i] = (uint16_t)(high << 8)+low;
  }

  sonar_data ret_data;
  ret_data.front = data[0];
  ret_data.front_left = data[1];
  ret_data.front_right = data [2];
  ret_data.back_left = data[3];
  ret_data.back_right = data[4];
  ret_data.back =  data[5];

  return ret_data;
}
