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
#define CYCLE_DURATION 50  
/* Informational value to roughly time the cycle duration (details in loop()) */
unsigned long cycle_start = 0;

/* Node handle - "that thingy that creates roserial nodes" */
ros::NodeHandle nh;

/* HW handles */ 
Motor_driver *md;
Sonar_driver *sd;
Battery_driver *bd;

/* Control flags */
bool measuring = false;

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

  pinMode(7, INPUT);
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
