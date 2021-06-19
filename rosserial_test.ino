/*************************************************/
/** Testovaci kod pro ovladani Trilobota 2.0     
 Podporuje:                                   
 * Jizdu (pevne dana vzdalenost i dokud se nerekne stop
      * Zpravy Motors_(continual_)move na spravne topicy (trilobot/motors/(continual_)move), 
        Empty msg na topic motors/stop              
 * Otaceni (opet o pevne dany uhel nebo dokud se nereknce stop)
      * Zpravy Motors_(continual_)turn.msg na motors/(cpontinual_)turn 
      * Empty msg na topic motors/stop
 * Jizda i otaceni davaji vedet o konci pohybu (jak v pripade kontinualniho, tak pevne daneho pohybu)
    na topic trilobot/motors/confirmation
 * Ziskavani dat ze 6 senzoru SRF-08 (zatim jen vzdalenost)     
      * Zadost pomoci Empty msg na topic trilobot/sonars/distance_request
      * Zasilany v msg Sonar_data na topic  trilobot/sonars/distance
**/

#include <ros.h>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include "motors.h"
const char topic_cmd_vel[] = "trilobot/cmd_vel";

ros::NodeHandle nh;

Motors *motors;
ros::Subscriber<geometry_msgs::Twist> sub_vel(topic_cmd_vel, &vel_callback);

float v_r = 0;
float v_l = 0;

volatile unsigned long ticks_r = 0;
volatile unsigned long ticks_l = 0;

// 64, 192 ... stop values
unsigned byte power_r = 64;
unsigned byte power_l = 192;

unsigned long start_time = 0; 
bool time_set = false;

float L = 0.2 (distance between wheels);
float eps = 0.0024; //
unsigned long cycle_start = 0;
bool moving = true; // TODO
#define VEL_MIN 0.02
void twist_callback(const geometry_msgs::Twist &msg)
{ 
  nh.loginfo("Message recieved");
  
  float v_l = msg.linear.x - (msg.angular.z*L)/2;
  float v_r = msg.linear.x + (msg.angular.z * L)/2;
  
  char result[8]; // Buffer big enough for 7-character float
  dtostrf(l, 6, 2, result); // Leave room for too large numbers!
  nh.loginfo(result);
  dtostrf(r, 6, 2, result); // Leave room for too large numbers!
  nh.loginfo(result);

  if(abs(v_l) < VEL_MIN)
  {
    v_l = 0;
  }

  if(abs(v_r) < VEL_MIN)
  {
    v_r = 0;
  }

  
  if (!time_set && (v_r || v_l) )
  {
    start_time = milis();
    time_set = true;
  }
  
  return;
}


void setup() {
  //Serial.begin(57600);
  motors = new Motors();
	nh.initNode();
  nh.loginfo("Motor movement done!");
 
  nh.subscribe(sub_vel);
}

#define CYCLE_DURATION 50

void loop() {
  cycle_start = milis();

  // motor part
  if(moving)
  {
    v_curr_r = (ticks_r * WHEEL_CIRCUIT/STEPS_ONE_CHANNEL)/((milis()-start_time)/1000.0);
    v_curr_l = (ticks_l * WHEEL_CIRCUIT/STEPS_ONE_CHANNEL)/((milis()-start_time)/1000.0);
    //R
    if(abs(v_curr_r-v_r) > eps)
    {
      if (v_curr_r - v_r > 0) //jedu moc rychle
      {
        power_r++; 
      }
      else if(v_curr_r - v_r < 0)
      {
        power_r--;
      }
    }
    motors->set_power('r', power_r);
    
    //L
     if(abs(v_curr_l-v_l) > eps)
      {
        if (v_curr_l - v_l > 0) //jedu moc rychle
        {
          power_l++; 
        }
        else if(v_curr_l - v_l < 0)
        {
          power_l--;
        }
      }
      motors->set_power('l', power_l);
    //vzorkovani rychlosti po -+ 100 ms
    start_time = (unsigned long) (milis()+start_time)/2;
    ticks_r = 0;
    ticks_l = 0;
    }
  
  nh.spinOnce();
  nh.loginfo("check");
  //NOTE: epxerimental value, can be something in range 1 to CYCLE_DURATION...  
  delay(cycle_start < CYCLE_DURATION ? CYCLE_DURATION - cycle_start : 1);
  //while(1);
}
