/*************************************************/
/** Testovaci kod pro ovladani Trilobota 2.0    **/
/*************************************************/

#define DEBUG_ENABLED 1

/* Generic includes */
/* nothing*/

/* ros includes */
#include <ros.h>  

/* Generic messages */
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

/* Trilobot specific messages - mostly structures of nums */
#include <trilobot/Odometry.h>
#include <trilobot/Sonar_data.h>
#include <trilobot/Battery_state.h>

/* Trilobot includes */ 
#include "motors.h"
#include "srf08.h"
#include "topics.h"

/* Basic definitions used through the whole file */

/* Messages */
trilobot::Sonar_data sonar_msg;
trilobot::Battery_state battery_msg;
trilobot::Odometry odometry_msg;

/* Node handle - "that thingy that creates roserial nodes" */
ros::NodeHandle nh;

/* HW handles */ 
Motors *motors;
Sonars *sonars;

/* Function declarations for easier overview */ 
ros::Subscriber<geometry_msgs::Twist> vel_sub(topic_cmd_vel, &vel_callback);
ros::Publisher odometry_pub(topic_trilobot_odometry, &odometry_msg);

ros::Subscriber<std_msgs::Empty> sonar_sub(topic_sonars_request, &sonars_callback);
ros::Publisher sonar_pub(topic_sonars_response, &sonar_msg);

ros::Publisher batt_pub(topic_battery_response, &battery_msg);
ros::Subscriber<std_msgs::Empty> batt_sub(topic_battery_request, &battery_callback);

/* Control flags */
bool time_set = false;
bool moving = true; // TODO
bool measuring = false;

unsigned long cycle_start = 0;
unsigned long measure_start = 0;

/* extern variables for odometry */
extern volatile unsigned long ticks_r;
extern volatile unsigned long ticks_l;

void sonars_callback(const std_msgs::Empty &msg)
{
  sonars->set_measurement();
  measuring = true;
  measure_start = millis();
}

void battery_callback(const std_msgs::Empty &msg)
{
  float batt0 = analogRead(A0)* 5.0/1023.0 * 147.0/100.0;
  float batt1 = analogRead(A1) * 5.0/1023.0 * 200.0/100.0 - batt0;
  float batt2 = analogRead(A2) * 5.0/1023.0 * 320.0/100.0 - batt0 - batt1;
  float batt3 = analogRead(A3) * 5.0/1023.0 * 400.0/100.0 - batt0 - batt1 - batt2;
  
  battery_msg.cell1 = batt0;
  battery_msg.cell2 = batt1;
  battery_msg.cell3 = batt2;
  battery_msg.cell4 = batt3;
  
  batt_pub.publish(&battery_msg);
}

void vel_callback(const geometry_msgs::Twist &msg)
{ 
  #if DEBUG_ENABLED
  char result[8];
  #endif

  motors->vel_l = msg.linear.x - (msg.angular.z*INTERWHEEL_DISTANCE)/2;
  motors->vel_r = msg.linear.x + (msg.angular.z * INTERWHEEL_DISTANCE)/2;

  
  if(abs(motors->vel_l) < MIN_VELOCITY)
  {
    motors->vel_l = 0;
  }

  if(abs(motors->vel_r) < MIN_VELOCITY)
  {
    motors->vel_r = 0;
  }
  
  if (!time_set && (motors->vel_r || motors->vel_l) )
  {
    motors->start_time_r = millis();
    motors->start_time_l = millis();
    ticks_r = 0;
    ticks_l = 0;
    time_set = true;
  }
  
  return;
}


void setup() {
  motors = new Motors();
  sonars = new Sonars();
	nh.initNode();
 
  nh.subscribe(vel_sub);
  nh.advertise(odometry_pub);

  nh.subscribe(sonar_sub);
  nh.advertise(sonar_pub);

  nh.subscribe(batt_sub);
  nh.advertise(batt_pub);


  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
}


#define CYCLE_DURATION 50 //ms
#define SRF08_MEASURE_TIME 70

void loop() 
{
  cycle_start = millis();
  
  // motor part
  if(moving)
  {
    odometry_msg.right = ticks_r;
    odometry_msg.left = ticks_l;
    odometry_pub.publish(&odometry.msg);
    motors->update();
    //char result[10]; // Buffer big enough for 7-character float
  }

  if(measuring)
  {
    if(millis() - measure_start >= SRF08_MEASURE_TIME) // 70 == cas od odeslani prikazu po zmereni
        {
          measuring= false;
          measure_start = millis();
          sonar_data data = sonars->get_distances();
          sonar_msg.front = data.front;
          sonar_msg.front_right = data.front_right;
          sonar_msg.front_left = data.front_left;
          sonar_msg.back_right= data.back_right;
          sonar_msg.back_left = data.back_left;
          sonar_msg.back = data.back;
          sonar_pub.publish(&sonar_msg);
        }
  }
  nh.spinOnce();


 // nh.("loginfocheck");
  //NOTE: epxerimental value, can be something in range 1 to CYCLE_DURATION...  
  delay((millis()-cycle_start) < CYCLE_DURATION ? CYCLE_DURATION - (millis()-cycle_start) : 1);
  //while(1);
}
