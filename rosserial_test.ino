/*************************************************/
/** Testovaci kod pro ovladani Trilobota 2.0    **/
/*************************************************/

#define DEBUG_ENABLED 1
#define EMERGENCY_BRAKE 1

/* Generic includes */
/* nothing*/

/* ros includes */
#include <ros.h>  

/* Generic messages */
#include <std_msgs/Empty.h>
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

#define CYCLE_DURATION 50 //ms
#define SRF08_MEASURE_TIME 70


/* Messages */
trilobot::Sonar_data sonar_msg;
trilobot::Battery_state battery_msg;
trilobot::Odometry odometry_msg;

/* Node handle - "that thingy that creates roserial nodes" */
ros::NodeHandle nh;
//ROS_master ros_master = new ROS_master(*nh);

/* HW handles */ 
Motor_driver *md;
Sonars *sonars;

/* Function declarations for easier overview */ 
ros::Publisher odometry_pub(topic_trilobot_odometry, &odometry_msg);

ros::Subscriber<std_msgs::Empty> sonar_sub(topic_sonars_request, &sonars_callback);
ros::Publisher sonar_pub(topic_sonars_response, &sonar_msg);

ros::Publisher batt_pub(topic_battery_response, &battery_msg);
ros::Subscriber<std_msgs::Empty> batt_sub(topic_battery_request, &battery_callback);

/* Control flags */
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
  if (EMERGENCY_BRAKE)
  {
  //TODO  pokud se bude blizit prekazka (distance < neco maleho), brzdi
  //md->emergency_stop();
  }
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




void setup() {
  nh.initNode();

  md = new Motor_driver(3*CYCLE_DURATION, &nh);
  
  sonars = new Sonars();
	
 
//  nh.subscribe(vel_sub);
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




void loop() 
{
  cycle_start = millis();
  
  // motor part - dont change md update and odometry sending!

  odometry_msg.right = ticks_r;
  odometry_msg.left = ticks_l;
  odometry_pub.publish(&odometry_msg);

  md->update();
  

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


  //NOTE: epxerimental value, can be something in range 1 to CYCLE_DURATION...  
  delay((millis()-cycle_start) < CYCLE_DURATION ? CYCLE_DURATION - (millis()-cycle_start) : 1);
}
