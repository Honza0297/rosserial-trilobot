/************************************************ 
Docking Station for Automatic Charging of Batteries of Robot  
File: battery.h                                 
Author: Jan Beran                             
Date: 2021-2022                             
This file is a part of authors diploma thesis 
                                                
Description: Declaration of the Battery_driver class which controls
              battery state and sends data to the Raspberry Pi.
**************************************************/
 
 #ifndef _BATTERY_H
 #define _BATTERY_H       1

/* ROS includes*/
#include <ros.h>
#include <trilobot/Sonar_data.h>
#include <std_msgs/Bool.h>
#include <trilobot/Battery_state.h>

/* Local includes*/
#include "topics.h"

#define BATT_UPDATE_INTERVAL 10000 // [ms]
#define CHARGE_UPDATE_INTERVAL 100  // [ms]

#define PIN_CELL0 A0
#define PIN_CELL1 A1
#define PIN_CELL2 A2
#define PIN_CELL3 A3

#define PIN_CHARGE_CHECK 7



class Battery_driver
{
  private:
    ros::NodeHandle *nh;

    ros::Publisher pub;
    ros::Publisher charge_pub;

    trilobot::Battery_state msg;
    std_msgs::Bool charge_msg;

    unsigned long last_update;
    unsigned long last_charge_update;

  public:
     Battery_driver(ros::NodeHandle *nh)
     : pub(topic_battery, &msg), charge_pub(topic_charging, &charge_msg)
     {
       this->nh = nh;
       this->nh->advertise(this->pub);
       this->nh->advertise(this->charge_pub);

       this->msg.cell1 = 0;
       this->msg.cell2 = 0;
       this->msg.cell3 = 0;
       this->msg.cell4 = 0;
       this->msg.charging = false;

       this->charge_msg.data = false;
       last_update = 0;
       last_charge_update = 0;

        /* A0-A3 are used to monitor voltage levels for each cell of the battery */
        pinMode(PIN_CELL0, INPUT);
        pinMode(PIN_CELL1, INPUT);
        pinMode(PIN_CELL2, INPUT);
        pinMode(PIN_CELL3, INPUT);
        
        /* Digital pin 7 is used to check charging status (charging/not charging)*/
        pinMode(PIN_CHARGE_CHECK, INPUT);

     }
     void update();

};
#endif
