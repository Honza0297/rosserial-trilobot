/*************************************************/
/** Testovaci kod pro ovladani Trilobota 2.0    **/
/*************************************************/

#define DEBUG_ENABLED 1

// generic includes
/* nothing*/

//ros includes
#include <ros.h>  

// Generic messages
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

//Trilobot specific messages - mostly structures of nums
#include <trilobot/Odometry.h>
#include <trilobot/Sonar_data.h>
#include <trilobot/Battery_state.h>

// trilobot includes
#include "motors.h"
#include "srf08.h"
#include "topics.h"

trilobot::Sonar_data sonar_msg;
trilobot::Battery_state battery_msg;
ros::NodeHandle nh;

Motors *motors;
Sonars *sonars;

ros::Subscriber<geometry_msgs::Twist> sub_vel(topic_cmd_vel, &vel_callback);
ros::Subscriber<std_msgs::Empty> sub_son_req(topic_sonars_request, &sonars_callback);
ros::Publisher sonar_pub(topic_sonars_response, &sonar_msg);
ros::Publisher batt_pub(topic_battery_response, &battery_msg);
ros::Subscriber<std_msgs::Empty> batt_sub(topic_battery_request, &battery_callback);

extern volatile unsigned long ticks_r;
extern volatile unsigned long ticks_l;
bool time_set = false;

float L = 0.2; //(distance between wheels)
float eps = 0.0024; //pripustna odchylka rychlost, 2,4 mm/s
unsigned long cycle_start = 0;
bool moving = true; // TODO
bool measuring = false;
unsigned long measure_start = 0;
#define VEL_MIN 0.02

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
  motors->v_l = msg.linear.x - (msg.angular.z*L)/2;
  motors->v_r = msg.linear.x + (msg.angular.z * L)/2;

  if(abs(motors->v_l) < VEL_MIN)
  {
    motors->v_l = 0;
  }

  if(abs(motors->v_r) < VEL_MIN)
  {
    motors->v_r = 0;
  }
  
  if (!time_set && (motors->v_r || motors->v_l) )
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
 
  nh.subscribe(sub_vel);
  nh.subscribe(sub_son_req);
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
