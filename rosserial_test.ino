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

#include <trilobot/Motors_turn.h>
#include <trilobot/Motors_move.h>
#include <trilobot/Motors_continual_move.h>
#include <trilobot/Motors_continual_turn.h>
#include <trilobot/Sonar_data.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include "motors.h"
#include "srf08.h"

const char topic_motors_move[] = "trilobot/motors/move";
const char topic_motors_continual_move[] = "trilobot/motors/continual_move";
const char topic_motors_turn[] = "trilobot/motors/turn";
const char topic_motors_continual_turn[] = "trilobot/motors/continual_turn";
const char topic_motors_stop[] = "trilobot/motors/stop";
const char topic_motors_confirmation[] = "trilobot/motors/confirmation";

const char topic_sonars_request[] = "trilobot/sonars/request";
const char topic_sonars_distance[] = "trilobot/sonars/distance";

ros::NodeHandle nh;

Motors *motors;
Sonars *sonars;

/* Msg pro potvrzovani*/
std_msgs::String str_msg;
char done_move[10] = "Move done";
char done_turn[10] = "Turn done";

trilobot::Sonar_data dst_msg;

ros::Publisher confirmer(topic_motors_confirmation, &str_msg);
ros::Publisher sonar_publisher();
sonar_publisher.advertise(topic_sonars_distance, &dst_msg, latch=true);

ros::Subscriber<trilobot::Motors_move> mover(topic_motors_move, &move_callback);
ros::Subscriber<trilobot::Motors_turn> turner(topic_motors_turn, &turn_callback);
ros::Subscriber<trilobot::Motors_continual_move> continual_mover(topic_motors_continual_move, &continual_move_callback);
ros::Subscriber<trilobot::Motors_continual_turn> continual_turner(topic_motors_continual_turn, &continual_turn_callback);
ros::Subscriber<std_msgs::Empty> stoper(topic_motors_stop, &stop_callback);

ros::Subscriber<std_msgs::Empty> sonar_sub(topic_sonars_request, &sonar_request_callback);

void sonar_request_callback(const std_msgs::Empty &msg)
{
  nh.loginfo("Got request to start sonaring");
  sonar_data data = sonars->get_distances();  
  
  dst_msg.front = data.front;
  dst_msg.front_right = data.front_right;
  dst_msg.front_left = data.front_left;
  dst_msg.back_right = data.back_right;
  dst_msg.back_left = data.back_left;
  dst_msg.back = data.back;

  sonar_publisher.publish(&dst_msg);
}

void stop_callback(const std_msgs::Empty &msg)
{
  motors->stop();
  nh.loginfo("Motor movement or turn done! (stopped continual movement/turn)");
  str_msg.data = done_move;
  confirmer.publish(&str_msg);
}
void move_callback(const trilobot::Motors_move &msg)
{
  motors->move(msg.distance, msg.speed);
  
  nh.loginfo("Motor movement done!");
  str_msg.data = done_move;
  confirmer.publish(&str_msg);
}

void continual_move_callback(const trilobot::Motors_continual_move &msg)
{
  motors->move(msg.speed);
}

void continual_turn_callback(const trilobot::Motors_continual_turn &msg)
{
  if(msg.direction == 'r')
  {
    motors->turn_right(msg.speed); 
  }
  else if(msg.direction == 'l')
   {
    motors->turn_left(msg.speed);
   }
  else
    return;
}

void turn_callback(const trilobot::Motors_turn &msg)
{
  motors->turn(msg.angle, msg.speed);
  
  nh.loginfo("Motor turning done!");
  str_msg.data = done_turn;
  confirmer.publish(&str_msg);
}



void setup() {
  motors = new Motors();
  sonars = new Sonars();
	nh.initNode();
  
  nh.advertise(confirmer);
  nh.advertise(sonar_publisher);
  
  nh.subscribe(mover);
  nh.subscribe(turner);
  nh.subscribe(continual_mover);
  nh.subscribe(continual_turner);
  nh.subscribe(stoper);
  nh.subscribe(sonar_sub);
}

void loop() {
  nh.spinOnce();
  delay(100);
}
