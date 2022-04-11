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

#define CYCLE_DURATION 50 //ms


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

unsigned long cycle_start = 0;

/* extern variables for odometry */
extern volatile unsigned long ticks_r;
extern volatile unsigned long ticks_l;








void setup() {
  nh.initNode();

  md = new Motor_driver(3*CYCLE_DURATION, &nh);
  sd = new Sonar_driver(&nh);
  bd = new Battery_driver(&nh);
  //sonars = new Sonar();
	
 
//  nh.subscribe(vel_sub);
  //nh.advertise(odometry_pub);

 // nh.subscribe(sonar_sub);
 // nh.advertise(sonar_pub);

//  nh.subscribe(batt_sub);
 // nh.advertise(batt_pub);


  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
}




void loop() 
{
  cycle_start = millis();
  
  // motor part - dont change md update and odometry sending!

  /*odometry_msg.right = ticks_r;
  odometry_msg.left = ticks_l;
  odometry_pub.publish(&odometry_msg);*/

  md->update();
  sd->update();
  

  nh.spinOnce();


  //NOTE: epxerimental value, can be something in range 1 to CYCLE_DURATION...  
  delay((millis()-cycle_start) < CYCLE_DURATION ? CYCLE_DURATION - (millis()-cycle_start) : 1);
}