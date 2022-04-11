 /************************************************ */
 /*  Educational tutorial for Arduino in robotics  */
 /*  File: srf08.h                                 */
 /*  Author: Jan Beran                             */
 /*  Date: autumn 2019                             */
 /* This file is a part of authors bachelor thesis */
 /*                                                */
 /* This file contains macros and function         */
 /* prototypes for controlling srf-08 ligh         */
 /* and distance sensor.                           */
 /**************************************************/
 
 #ifndef _BATTERY_H
 #define _BATTERY_H       1

#include "topics.h"
#include <ros.h>
#include <trilobot/Sonar_data.h>
#include <std_msgs/Empty.h>
#include <trilobot/Battery_state.h>

class Battery_driver
{
  private:
    ros::NodeHandle *nh;

    ros::Publisher pub;
    ros::Subscriber<std_msgs::Empty, Battery_driver> sub;
    trilobot::Battery_state msg;

    void callback(const std_msgs::Empty &msg);

  public:
     Battery_driver(ros::NodeHandle *nh)
     : pub(topic_battery_response, &msg), sub(topic_battery_request, &Battery_driver::callback, this)
     {
       this->nh = nh;
       this->nh->subscribe(this->sub);
       this->nh->advertise(this->pub);
       this->msg.cell1 = 0;
       this->msg.cell2 = 0;
       this->msg.cell3 = 0;
       this->msg.cell4 = 0;
     }
     void update();

};
#endif
