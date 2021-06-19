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

bool backward_r = false;
bool backward_l = false;

extern volatile unsigned long ticks_r;
extern volatile unsigned long ticks_l;

// 64, 192 ... stop values
byte power_r = 64;
byte power_l = 192;

unsigned long start_time_r = 0;
unsigned long start_time_l = 0; 
bool time_set = false;

float L = 0.2; //(distance between wheels)
float eps = 0.0024; //pripustna odchylka rychlost, 2,4 mm/s
unsigned long cycle_start = 0;
bool moving = true; // TODO
#define VEL_MIN 0.02

void vel_callback(const geometry_msgs::Twist &msg)
{ 
  //nh.loginfo("Message recieved");
  
  float v_l = msg.linear.x - (msg.angular.z*L)/2;
  float v_r = msg.linear.x + (msg.angular.z * L)/2;
  
  /*char result[8]; // Buffer big enough for 7-character float
  dtostrf(v_l, 6, 2, result); // Leave room for too large numbers!
  nh.loginfo(result);
  dtostrf(v_r, 6, 2, result); // Leave room for too large numbers!
  nh.loginfo(result);*/

  if(abs(v_l) < VEL_MIN)
  {
    v_l = 0;
  }

  if(abs(v_r) < VEL_MIN)
  {
    v_r = 0;
  }
  
  if (v_r < 0)
  {
    backward_r = true;
  }
  else
  {
    backward_r = false;
  }

  if (v_l < 0)
  {
    backward_l = true;
  }
  else
  {
    backward_l = false;
  }


  motors->v_l = v_l;
  motors->v_r = v_r;
  
  if (!time_set && (v_r || v_l) )
  {
    start_time_r = millis();
    start_time_l = millis();
    ticks_r = 0;
    ticks_l = 0;
    time_set = true;
  }
  
  return;
}


void setup() {
  //Serial.begin(57600);
  motors = new Motors();
	nh.initNode();
  //nh.loginfo("Motor movement done!");
 
  nh.subscribe(sub_vel);
}

#define CYCLE_DURATION 100
void loop() {
  cycle_start = millis();
  
  // motor part
  if(moving)
  {char result[10]; // Buffer big enough for 7-character float

  /*  double delta = (double) (millis()-start_time_r) / 1000.0 ;
    nh.loginfo("time:");
    String(delta, 4).toCharArray(result, 8);
    nh.loginfo(result);*/
    double v_curr_r =( (double) (( ticks_r * 0.279)/768.0 )/( (double) (millis()-start_time_r) / 1000.0) );
    ticks_r = 0;
    start_time_r= millis();   
    /*unsigned long tr = ticks_r;
    ticks_r = 0;
    start_time_r= millis();   
    nh.loginfo("tr:");
    sprintf(result,"%lu",tr);
    nh.loginfo(result);
    nh.loginfo("tr*wc:");
    String(( (double)tr * 0.279),3).toCharArray(result, 8);
    nh.loginfo(result);

    nh.loginfo("tr*wc/soc:");
    float x = (float)tr*0.279/768.0;
    sprintf(result, "%f", x);
    nh.loginfo(result);

    nh.loginfo("tr*wc/soc/time:");
    String(( (double) (( (double)tr * WHEEL_CIRCUIT)/(double)STEPS_ONE_CHANNEL )/( (double) (millis()-start_time_l) / 1000.0) ),5).toCharArray(result, 8);
    nh.loginfo(result);
    nh.loginfo("vcr:");
    String(v_curr_r, 5).toCharArray(result, 8);
    nh.loginfo(result);*/
    

    
    double v_curr_l = ( (double) (( (double)ticks_l * WHEEL_CIRCUIT)/(double)STEPS_ONE_CHANNEL )/( (double) (millis()-start_time_l) / 1000.0) );
    ticks_l = 0;
    start_time_l = millis();
    /*nh.loginfo("ticks_r is:");
    String(ticks_r).toCharArray(result, 8);
    nh.loginfo(result);
    nh.loginfo("v_curr_r is:");
    String(v_curr_r).toCharArray(result, 8);
    nh.loginfo(result);*/
    
    //R
    if(abs(v_curr_r-motors->v_r) > eps)
    {
      if (v_curr_r - motors->v_r > 0) 
      {
        if(backward_r)
        {
          if(power_r <= 64)
          {
             power_r = power_r == 127 ? power_r : power_r+1; 
          }
          else
          {
            power_r = power_r == 1 ? power_r : power_r-1;
          }
        }
        else
        {
          if(power_r >= 64)
          {
            power_r = power_r == 1 ? power_r : power_r-1;
          }
          else
          {
            power_r = power_r == 127 ? power_r : power_r+1; 
          }
          
        }
      }
      else if(v_curr_r - motors->v_r < 0)
      {
        if(backward_r)
        {
          power_r = power_r == 127 ? power_r : power_r+1; 
        }
        else
        {
          power_r = power_r == 1 ? power_r : power_r-1;
         
        }
      }
    }
    motors->set_power('r', power_r);
   /* String(power_r).toCharArray(result, 8);
    nh.loginfo("power is:");
    nh.loginfo(result);*/
    //L
     if(abs(v_curr_l-motors->v_l) > eps)
      {
        if (v_curr_l - motors->v_l > 0) //jedu moc rychle
        {
        if(backward_l)
        {
          if(power_l <= 192)
          {
             power_l = power_l == 127 ? power_l : power_l+1; 
          }
          else
          {
            power_l = power_l == 1 ? power_l : power_l-1;
          }
        }
        else
        {
          if(power_l >= 192)
          {
            power_l = power_l == 1 ? power_l : power_l-1;
          }
          else
          {
            power_l = power_l == 127 ? power_l : power_l+1; 
          }
          
        }
        }
        else if(v_curr_l - motors->v_l < 0)
        {
          if(backward_l)
          {
            power_l = power_l == 255 ? power_l : power_l+1; 
          }
          else
          {
            power_l = power_l == 128 ? power_l : power_l -1;
          }
        }
      }
      motors->set_power('l', power_l); //TODO
      
    //vzorkovani rychlosti po -+ 100 ms
    
    //ticks_r = 0;
   // ticks_l = 0;
    }
  
  nh.spinOnce();
 // nh.loginfo("check");
  //NOTE: epxerimental value, can be something in range 1 to CYCLE_DURATION...  
  delay((millis()-cycle_start) < CYCLE_DURATION ? CYCLE_DURATION - (millis()-cycle_start) : 1);
  //while(1);
}
