 /************************************************ */
 /*  Educational tutorial for Arduino in robotics  */
 /*  Vyukovy Tutorial pro pouziti Arduina v robotice*/
 /*  File: srf08.cpp                               */
 /*  Author: Jan Beran                             */
 /*  Date: autumn 2019                             */
 /*                                                */
 /* This file is a part of author´s bachelor thesis*/
 /*                                                */
 /**************************************************/

#include <Arduino.h>
#include "battery.h"
#include <std_msgs/Empty.h>

void Battery_driver::callback(const std_msgs::Empty &msg)
{

  float batt0 = analogRead(A0)* 5.0/1023.0 * 147.0/100.0;
  float batt1 = analogRead(A1) * 5.0/1023.0 * 200.0/100.0 - batt0;
  float batt2 = analogRead(A2) * 5.0/1023.0 * 320.0/100.0 - batt0 - batt1;
  float batt3 = analogRead(A3) * 5.0/1023.0 * 400.0/100.0 - batt0 - batt1 - batt2;
  
  this->msg.cell1 = batt0;
  this->msg.cell2 = batt1;
  this->msg.cell3 = batt2;
  this->msg.cell4 = batt3;
  
  this->pub.publish(&this->msg);
}
