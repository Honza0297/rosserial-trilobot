/*************************************************/
/** Testovaci kod pro ovladani Trilobota 2.0    **/
/*************************************************/

#define DEBUG_ENABLED 1

/* Generic includes */
/* --empty-- */

/* ROS generic includes*/
#include <ros.h>  

/* Generic messages */
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>

/* Trilobot includes */ 
#include "motors.h"
#include "srf08.h"
#include "topics.h"
#include "battery.h"

/* Basic definitions used through the whole file */

/* How long should one cycle take (at least) */
#define CYCLE_DURATION 20 // [ms]
#define CMD_VEL_PERIOD 100 //[ms]
#define CMD_VEL_TIMEOUT 3*CMD_VEL_PERIOD /*NOTE:nutne otestovat, momentalne 3.33 zpravy za sec, nez se zacne sekat */
/* Informational value to roughly time the cycle duration (details in loop()) */
unsigned long cycle_start = 0;
/* Master = RPi rosserial site */
bool master_running = false;
unsigned long last_master_ping = 0;
#define MASTER_TIMEOUT 2000 //ms
/* Node handle - "that thingy that creates roserial nodes" */
ros::NodeHandle nh;

void callback(const std_msgs::Empty& msg)
{
  master_running = true;
  last_master_ping = millis();
}
ros::Subscriber<std_msgs::Empty> sub("trilobot/rosserial_start", &callback);



/* HW handles */ 
Motor_driver *md;
Sonar_driver *sd;
Battery_driver *bd;

/* Control flags */
bool measuring = false;

void setup() {

  nh.initNode();
  nh.subscribe(sub);
  
  md = new Motor_driver(CMD_VEL_TIMEOUT, &nh);
  sd = new Sonar_driver(&nh);
  bd = new Battery_driver(&nh);
 

}


void loop() 
{
  /* Check whether Raspberry explicitly enables Arduino */
  while(!master_running)
  {
    nh.spinOnce();
  }

  /* if master did not send sync for more than MASTER_TIMEOUT ms, shut down updates (mostly sonars measurements) */
  if(millis()-last_master_ping >= MASTER_TIMEOUT)
  {
    master_running = false;
  }
  
  
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
