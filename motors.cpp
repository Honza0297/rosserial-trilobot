 /************************************************ */
 /*  Educational tutorial for Arduino in robotics  */
 /*  Vyukovy Tutorial pro pouziti Arduina v robotice*/
 /*  File: motors.cpp                              */
 /*  Author: Jan Beran                             */
 /*  Date: autumn 2019                             */
 /*                                                */
 /* This file is a part of author´s bachelor thesis*/
 /*                                                */
 /**************************************************/


#include <Arduino.h>
#include "motors.h"
#include "topics.h"
#include <geometry_msgs/Twist.h>

/**
 * Promenne nutne pro spravne fungovani preruseni.
 * */
volatile unsigned long ticks_r = 0;
volatile unsigned long ticks_l = 0; 

/**
 * Prototypy funkci pro ovladani preruseni
 * */
void motor_right_interrupt_handler();
void motor_left_interrupt_handler();
void attach_interrupts();
void detach_interrupts();

/*Motor_driver::Motor_driver(int timeout, ros::NodeHandle &nh)
{
  this->nh = nh;
  this->timeout = timeout;
  this->motors = new Motors();
  this->set_desired_speed(0,0);
  this->timestamp_r = 0;
  this->timestamp_l = 0;
  //this->vel_sub(topic_cmd_vel, &Motor_driver::vel_callback, &this);

  this->nh.subscribe(this->vel_sub);

}*/

void Motor_driver::emergency_stop()
{
  this->motors->stop();
  this->stop();
}

void Motor_driver::stop()
{
  this->set_desired_speed(0,0);
}

void Motor_driver::vel_callback(const geometry_msgs::Twist &msg)
{ 
  this->nh->loginfo("ddD");
  float speed_l = msg.linear.x - (msg.angular.z*INTERWHEEL_DISTANCE)/2;
  float speed_r = msg.linear.x + (msg.angular.z * INTERWHEEL_DISTANCE)/2;
  this->set_desired_speed(speed_l, speed_r);

  return;
}

void Motor_driver::set_desired_speed(float l, float r)
{
  this->last_update = millis();

   if(abs(l) < MIN_VELOCITY)
  {
    this->desired_speed_l = 0;
  }
  else
  {
    this->desired_speed_l = l;
  }

  if(abs(r) < MIN_VELOCITY)
  {
    this->desired_speed_r = 0;
  }
  else
  {
    this->desired_speed_r = r;
  }


 /*if (motors->vel_r || motors->vel_l)//(!time_set && (motors->vel_r || motors->vel_l) )
  {
    this->timestamp_l = millis();
    this->timestamp_r = millis();
    ticks_r = 0; //WTF
    ticks_l = 0; //WTF 
   // time_set = true;
  }*/
}


void Motor_driver::update()
{
 /* if (this->desired_speed_l == 0 && this->desired_speed_r == 0){}*/

  this->dum.odometry_msg.right = ticks_r;
  this->dum.odometry_msg.left= ticks_l;
  this->dum.pub.publish(&this->dum.odometry_msg);
  
  //if no update for too long, stop
  if((millis() - this->last_update) >= this->timeout)
  {
    this->desired_speed_r = 0;
    this->desired_speed_l = 0;
  }    

  float current_speed_r = this->motors->get_dir_coef(this->motors->power_r) * ((double)(((ticks_r - this->last_ticks_r) * 0.279)/768.0) / ((double)(millis()-this->timestamp_r) / 1000.0));
  //ticks_r = 0;
  this->last_ticks_r = ticks_r;
  this->timestamp_r = millis();
  
  
  byte power_r = this->motors->get_power('r');
  if (this->desired_speed_r == 0 && abs(this->motors->power_r - POWER_STOP_R) < 3)
  {
    power_r = POWER_STOP_R;
  }
  else
  {
    if(this->desired_speed_r > current_speed_r)
    {
      power_r = power_r < 127 ? power_r+1 : power_r;
      //TODO loguj pokud jedes na max, ale stejne je to malo (druhy case)
    }
    if(this->desired_speed_r < current_speed_r)
    {
      power_r = power_r > 1 ? power_r-1 : power_r;
      //todo log, ze pomaleji to uz nepujde v pripade druheho pripadu
    }
  }

  float current_speed_l = this->motors->get_dir_coef(this->motors->power_l) * 
                          (
                            (double)(((ticks_l - this->last_ticks_l) * 0.279)/768.0) / ((double)(millis()-this->timestamp_l) / 1000.0) 
                          );
  this->last_ticks_l = ticks_l;
  this->timestamp_l = millis();
  
  
  byte power_l = this->motors->get_power('l');
  if (this->desired_speed_l == 0 && abs(this->motors->power_l - POWER_STOP_L) < 3)
  {
    power_l = POWER_STOP_L;
  }
  else
  {
    if(this->desired_speed_l > current_speed_l)
    {
      power_l = power_l < 255 ? power_l+1 : power_l;
      //TODO loguj pokud jedes na max, ale stejne je to malo (druhy case)
    }
    if(this->desired_speed_l < current_speed_l)
    {
      power_l = power_l > 128 ? power_l-1 : power_l;
      //todo log, ze pomaleji to uz nepujde v pripade druheho pripadu
    }
  }

  this->motors->set_power(power_l, power_r);
}       



void Motors::update()
{
    double v_curr_r = this->get_dir_coef(this->power_r) * ( (double) (( ticks_r * 0.279)/768.0 )/( (double) (millis()-this->start_time_r) / 1000.0) );
    ticks_r = 0;
    this->start_time_r= millis();   
    if (this->vel_r == 0 && abs(this->power_r - 64) < 3)
    {
      this->power_r = 64;
    }
    else
    {
      if(this->vel_r > v_curr_r)
      {
        if(this->power_r < 127)
        {
          this->power_r++;
        }
        //TODO else loguj, ze jedes na max, ale stejne malo
      }
      if(this->vel_r < v_curr_r)
      {
        if(this->power_r > 1)
        {
          this->power_r--;
        }
        //todo log, ze pomaleji to uz nepujde
      }
    }
        
    double v_curr_l = get_dir_coef(this->power_l)*( (double) (( ticks_l * 0.279)/768.0 )/( (double) (millis()-this->start_time_l) / 1000.0) );
    ticks_l = 0;
    this->start_time_l = millis();
    if(this->vel_l == 0 && abs(this->power_l-192) < 3)
    {
      this->power_l = 192;    
    }
    else
    {
      if(this->vel_l > v_curr_l)
      {
        if(this->power_l < 255)
        {
          this->power_l++;
        }
        //TODO else loguj, ze jedes na max, ale stejne malo
      }
      if(this->vel_l < v_curr_l)
      {
        if(this->power_l > 128)
        {
          this->power_l--;
        }
        //todo log, ze pomaleji to uz nepujde
      }

    }
     // this->set_power('r');
      //this->set_power('l');   
}

byte Motors::get_power(char motor)
{
  byte ret = 0;
  switch(motor)
  {
    case 'r':
      ret = this->power_r;
      break;
    case 'l':
      ret = this->power_l;
      break;
    default:
    {}
  }
  return ret;
}

Motors::Motors()
{
  pinMode(LEFT_A, INPUT);
  pinMode(LEFT_B, INPUT);
  pinMode(RIGHT_A, INPUT);
  pinMode(RIGHT_B, INPUT);
  
//  this->vel_l = 0;
//  this->vel_r = 0;
  this->power_r = POWER_STOP_R;
  this->power_l = POWER_STOP_L;
  this->start_time_r = 0;
  this->start_time_l = 0; 
  Serial1.begin(MOTOR_BAUDRATE);
  
  this->stop();
  attach_interrupts();
  //this->move_in_progress = false;
  //this->steps_to_go = -1; 
  
}

bool Motors::moving()
{
  bool ret;

  if (power_l == POWER_STOP_L || this->power_r == POWER_STOP_R)
  {
    ret = false;
  }
  else
  {
    ret = true;
  }

  return ret;

}

void Motors::set_power(byte l, byte r)
{
  this->power_l = l;
  this->power_r = r;

  Serial1.write(this->power_r);
  Serial1.write(this->power_l);
   
}

int Motors::get_dir_coef(byte power)
{
  int retval = 0;
  
  if((power > 0 && power < 64)
      ||
     (power > 127 && power < 192))
  {
    retval = -1;
  }
  else if((power > 64 && power < 128)
      ||
     (power > 192 && power < 256))
  {
    retval = 1;  
  }
  else // power == 64 || 192
  {
    retval = 0;
  }
    
  return retval;
}



void Motors::stop()
{
  Serial1.write(STOP_BYTE);
}



/************************************/
/* Funkce pro ovladni preruseni     */
/************************************/

/** @brief Funkce pro obsluhu preruseni z praveho motoru */
void motor_right_interrupt_handler()
{
  ticks_r++;
}

/** @brief Funkce pro obsluhu preruseni z leveho motoru */
void motor_left_interrupt_handler()
{
  ticks_l++;
}

/** @brief Funkce zapne preruseni na pinech enkoderu */
void attach_interrupts()
{
 attachInterrupt(digitalPinToInterrupt(RIGHT_A), motor_right_interrupt_handler, RISING);
 attachInterrupt(digitalPinToInterrupt(LEFT_A), motor_left_interrupt_handler, RISING);
}

/** @brief Funkce vypne preruseni na pinech enkoderu a vynuluje citaci poctu kroku. */
void detach_interrupts()
{
 detachInterrupt(digitalPinToInterrupt(LEFT_A));
 detachInterrupt(digitalPinToInterrupt(RIGHT_A));
}
