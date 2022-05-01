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
#include <trilobot/Vel.h>

/**
 * Promenne nutne pro spravne fungovani preruseni.
 * NOTE: v celem kodu nejsou nulovany, protoze nez unsigned long pretece, Trilobot ujede cca 1400 km :)
 * TODO: Nulovat pro zasilani do RPi 
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


void Motor_driver::emergency_stop()
{
  this->motors->stop();
  this->stop();
}

void Motor_driver::stop()
{
  this->set_goal_speed(0,0);
}

void Motor_driver::vel_callback(const trilobot::Vel &msg)
{ 
  //this->nh->loginfo("ddD");
  float speed_l = msg.x - (msg.theta*INTERWHEEL_DISTANCE)/2;
  float speed_r = msg.x + (msg.theta * INTERWHEEL_DISTANCE)/2;
  this->set_goal_speed(speed_l, speed_r);

  return;
}

void Motor_driver::set_goal_speed(float l, float r)
{
  this->last_update = millis();

   if(abs(l) < MIN_VELOCITY)
  {
    this->goal_speed_l = 0;
  }
  else
  {
    this->goal_speed_l = l;
  }

  if(abs(r) < MIN_VELOCITY)
  {
    this->goal_speed_r = 0;
  }
  else
  {
    this->goal_speed_r = r;
  }
}


void Motor_driver::update()
{

  /* TODO if speed is zero for both motors, dont publish odometry because of bandwidth*/
  this->msg.r = ticks_r;

  /*char bug[15];
  sprintf(bug, "%lu\n", ticks_r);
  this->nh->loginfo(bug);*/
  if(this->motors->power_r > POWER_STOP_R)
  {
    this->msg.rdir = DIR_FORW;
  }
  else if(this->motors->power_r < POWER_STOP_R)
  {
    this->msg.rdir = DIR_BACK;
  }
  else
  {
    this->msg.rdir = DIR_NONE;
  }

  this->msg.l = ticks_l;
  if(this->motors->power_l > POWER_STOP_L)
  {
    this->msg.ldir = DIR_FORW;
  }
  else if(this->motors->power_l < POWER_STOP_L)
  {
    this->msg.ldir = DIR_BACK;
  }
  else
  {
    this->msg.ldir = DIR_NONE;
  }

  
  //this->dir_pub.publish(&this->dir_msg);
  this->pub.publish(&this->msg);
  
  //if no update for too long, soft stop
  if((millis() - this->last_update) >= this->timeout)
  {
    this->goal_speed_r = 0;
    this->goal_speed_l = 0;
  }    

  byte power_r = this->compute_new_power('r');
  byte power_l = this->compute_new_power('l');

  this->motors->set_power(power_l, power_r);
}       

byte Motor_driver::compute_new_power(char motor)
{
  /*
    TODO:
    - make function for computing current speed
    - make magic values constants
    **/
  byte power;
  if(motor == 'r')
  {
    float current_speed_r = this->motors->get_dir_coef(this->motors->power_r) * ((double)(((ticks_r - this->last_ticks_r) * 0.279)/768.0) / ((double)(millis()-this->timestamp_r) / 1000.0));
    this->last_ticks_r = ticks_r;
    this->timestamp_r = millis();
    
    
    byte power_r; 
    /* Final stop from very low speed */
    if (this->goal_speed_r == 0 && abs(this->motors->power_r - POWER_STOP_R) < MIN_POWER)
    {
      power_r = POWER_STOP_R;
    }
    else
    {
      power_r = this->motors->get_current_power('r'); /* copy current power */
      if(this->goal_speed_r > current_speed_r)
      {
        power_r = power_r < 127 ? power_r+1 : power_r;
        //TODO loguj pokud jedes na max, ale stejne je to malo (druhy case)
      }
      if(this->goal_speed_r < current_speed_r)
      {
        power_r = power_r > 1 ? power_r-1 : power_r;
        //todo log, ze pomaleji to uz nepujde v pripade druheho pripadu
      }
    }
    power = power_r;
  }
  else if (motor == 'l')
  {
    float current_speed_l = this->motors->get_dir_coef(this->motors->power_l) * 
                          (
                            (double)(((ticks_l - this->last_ticks_l) * 0.279)/768.0) / ((double)(millis()-this->timestamp_l) / 1000.0) 
                          );
    this->last_ticks_l = ticks_l;
    this->timestamp_l = millis();

    byte power_l; 
    /* Final stop from very low speed */
    if (this->goal_speed_l == 0 && abs(this->motors->power_l - POWER_STOP_L) < MIN_POWER)
    {
      power_l = POWER_STOP_L;
    }
    else
    {
      power_l = this->motors->get_current_power('l');
      if(this->goal_speed_l > current_speed_l)
      {
        power_l = power_l < 255 ? power_l+1 : power_l;
        //TODO loguj pokud jedes na max, ale stejne je to malo (druhy case)
      }
      if(this->goal_speed_l < current_speed_l)
      {
        power_l = power_l > 128 ? power_l-1 : power_l;
        //todo log, ze pomaleji to uz nepujde v pripade druheho pripadu
      }
    }
    power = power_l;
  }

  return power;
}


byte Motors::get_current_power(char motor)
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
  
  this->power_r = POWER_STOP_R;
  this->power_l = POWER_STOP_L;
  this->start_time_r = 0;
  this->start_time_l = 0; 
  Serial1.begin(MOTOR_BAUDRATE);
  
  this->stop();
  attach_interrupts();  
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
