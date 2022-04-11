/*************************************************/
/** Testovaci kod pro ovladani Trilobota 2.0    **/
/*************************************************/

#define DEBUG_ENABLED 1

/* Generic includes */
/* nothing*/

/* ros includes */
#include <ros.h>  

/* Generic messages */
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>

/* Trilobot specific messages - mostly structures of nums */
//#include <trilobot/Odometry.h>

/* Trilobot includes */ 
#include "motors.h"
#include "srf08.h"
#include "topics.h"
#include "battery.h"
/* Basic definitions used through the whole file */

/* How long should one cycle take (at least)*/
#define CYCLE_DURATION 50

/* Informational value to roughly time the cycle duration (details in loop()) */
unsigned long cycle_start = 0;


/* Messages */
//trilobot::Sonar_data sonar_msg;
//trilobot::Odometry odometry_msg;

/* Node handle - "that thingy that creates roserial nodes" */
ros::NodeHandle nh;
//ROS_master ros_master = new ROS_master(*nh);

/* HW handles */ 
Motor_driver *md;
Sonar_driver *sd;
Battery_driver *bd;
//Sonar *sonars;

/* Function declarations for easier overview */ 
//ros::Publisher odometry_pub(topic_trilobot_odometry, &odometry_msg);

//ros::Subscriber<std_msgs::Empty> sonar_sub(topic_sonars_request, &sonars_callback);
//ros::Publisher sonar_pub(topic_sonars_response, &sonar_msg);

//ros::Publisher batt_pub(topic_battery_response, &battery_msg);
//ros::Subscriber<std_msgs::Empty> batt_sub(topic_battery_request, &battery_callback);

/* Control flags */
bool measuring = false;

/* Extern variables for odometry */
extern volatile unsigned long ticks_r;
extern volatile unsigned long ticks_l;


void setup() {

  nh.initNode();

  md = new Motor_driver(3*CYCLE_DURATION, &nh);
  sd = new Sonar_driver(&nh);
  bd = new Battery_driver(&nh);
 
  /* A0-A3 are used to monitor voltage levels for each cell of the battery */
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
}




void loop() 
{
  cycle_start = millis();

  /* Motor driver udpate */
  md->update();

  /* Sonar driver udpate */
  sd->update();

  /* Battery driver update */
  bd->update();

  nh.spinOnce();

 /* Rough timing of one cycle.
    It guarantees minimal cycle duration (set by CYCLE_DURATION), but not the maximal duration! */
 delay((millis()-cycle_start) < CYCLE_DURATION ? (CYCLE_DURATION - (millis()-cycle_start)) : 1);
}
