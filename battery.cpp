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
  this->pub.publish(&this->msg);
}

void Battery_driver::update()
{
  if((millis() - this->last_update)) >= BATT_UPDATE_INTERVAL)
  {
    this->last_update = millis();

    /* Get battery voltages. 
      First fraction (5/1023) is conversion from range of ADC to 0-5 [V]
      Second one is a formula for voltage dividers (see documentation/diploma thesis) */
    this->msg.cell1 = analogRead(A0)* 5.0/1023.0 * 147.0/100.0; 
    this->msg.cell2 = analogRead(A1) * 5.0/1023.0 * 200.0/100.0 - this->msg.cell1;
    this->msg.cell3 = analogRead(A2) * 5.0/1023.0 * 320.0/100.0 - this->msg.cell1 - this->msg.cell2;
    this->msg.cell4 = analogRead(A3) * 5.0/1023.0 * 400.0/100.0 - this->msg.cell1 - this->msg.cell2 - this->msg.cell3;

    this->pub.publish(&this->msg);
  }
}
