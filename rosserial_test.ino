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
ros::Subscriber<geometry_msgs::Twist> newmover(topic_cmd_vel, &twist_callback);

float L = 0.2;


void twist_callback(const geometry_msgs::Twist &msg)
{ nh.loginfo("Message recved");
  float l = msg.linear.x - (msg.angular.z*L)/2;
  float r = msg.linear.x + (msg.angular.z * L)/2;
  char result[8]; // Buffer big enough for 7-character float
  dtostrf(l, 6, 2, result); // Leave room for too large numbers!
  nh.loginfo(result);
  dtostrf(r, 6, 2, result); // Leave room for too large numbers!
  nh.loginfo(result);
  motors->newmove(l,r);
}


void setup() {
  motors = new Motors();
  
	nh.initNode();
  nh.loginfo("Motor movement done!");
 
  nh.subscribe(newmover);
}

void loop() {
  nh.spinOnce();
  nh.loginfo("check");
  // NOTE: epxerimental value, could be something in range 10-100...
  delay(100);
}
