/****************
Docking Station for Automatic Charging of Batteries of Robot

Author: Jan Beran
File: topics.h
Date: 2021-2022
Description: ROS topics for sending data from Arduino to Raspberry Pi via rosserial.
****************************/
#ifndef _TOPICS_H 
#define _TOPICS_H 1

const char topic_cmd_vel[] = "trilobot/cmd_vel";
const char topic_sonar_data[] = "trilobot/sonar_data";
const char topic_battery[] = "trilobot/battery_raw";
const char topic_charging[] = "trilobot/battery_charging";
const char topic_trilobot_odometry[] = "trilobot/odometry";
const char topic_trilobot_direction[] = "trilobot/direction";
#endif
