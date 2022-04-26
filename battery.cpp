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

void Battery_driver::update()
{
  if((millis() - this->last_update) >= BATT_UPDATE_INTERVAL)
  {
    this->last_update = millis();

    /* Get battery voltages. 
      First fraction (5/1023) is conversion from range of ADC to 0-5 [V]
      Second one is a formula for voltage dividers (see documentation/diploma thesis) */
    this->msg.cell1 = 0.0;// analogRead(PIN_CELL0)* 5.0/1023.0 * 147.0/100.0; 
    this->msg.cell2 = 1.0;//analogRead(PIN_CELL1) * 5.0/1023.0 * 200.0/100.0 - this->msg.cell1;
    this->msg.cell3 = 2.0;//analogRead(PIN_CELL2) * 5.0/1023.0 * 320.0/100.0 - this->msg.cell1 - this->msg.cell2;
    this->msg.cell4 = 3.0;//analogRead(PIN_CELL3) * 5.0/1023.0 * 400.0/100.0 - this->msg.cell1 - this->msg.cell2 - this->msg.cell3;
    /* NOTE: rosserial on the RPi site somehow managed to "cache" the merged version of the message (voltages AND charge status).
       In other words, after splitting it to two messages, rosserial still insists that it needs charge status even here. 
       After DP submission, it would be good to fix that, however, it is not a painfull problem (sending just 1B/10s should not bother bandwith too much :) ) */
    this->charge_msg.data = digitalRead(PIN_CHARGE_CHECK) == 0 ? false : true ;
    this->pub.publish(&this->msg);
  }

  if((millis() - this->last_charge_update) >= CHARGE_UPDATE_INTERVAL)
    {
      this->last_charge_update = millis();

      this->charge_msg.data = digitalRead(PIN_CHARGE_CHECK) == 0 ? false : true ;

      this->charge_pub.publish(&this->charge_msg);
    }

}
