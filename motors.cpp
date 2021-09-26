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


/**
 * Promenne nutne pro spravne fungovani preruseni.
 * */

volatile unsigned long ticks_r = 0;
volatile unsigned long ticks_l = 0; 


volatile int steps_right = 0;
volatile int steps_left = 0;
volatile double time_left = 0;
volatile double time_right = 0;

float delta = 0.01; 
float speed_left = 0;
float speed_right = 0;
/**
 * Prototypy funkci pro ovladani preruseni
 * */
void motor_right_interrupt_handler();
void motor_left_interrupt_handler();
void attach_interrupts();
void detach_interrupts();

Motors::Motors()
{
  pinMode(LEFT_A, INPUT);
  pinMode(LEFT_B, INPUT);
  pinMode(RIGHT_A, INPUT);
  pinMode(RIGHT_B, INPUT);
  
  this->v_l = 0;
  this->v_r = 0;
  this->power_r = 64;
  this->power_l = 192;
  this->start_time_r = 0;
  this->start_time_l = 0; 
  Serial1.begin(MOTOR_BAUDRATE);
  
  this->stop();
  attach_interrupts();
  //this->move_in_progress = false;
  //this->steps_to_go = -1; 
  
}
void Motors::set_power(char motor)
{
  switch (motor)
  {
    case 'r':
      Serial1.write(this->power_r);
      break;
    case 'l':
      Serial1.write(this->power_l);
      break;
    default:
    {}
  }
  return;  
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

void Motors::update()
{
    double v_curr_r = this->get_dir_coef(this->power_r) * ( (double) (( ticks_r * 0.279)/768.0 )/( (double) (millis()-this->start_time_r) / 1000.0) );
    ticks_r = 0;
    this->start_time_r= millis();   
    if (this->v_r == 0 && abs(this->power_r - 64) < 3)
    {
      this->power_r = 64;
    }
    else
    {
      if(this->v_r > v_curr_r)
      {
        if(this->power_r < 127)
        {
          this->power_r++;
        }
        //TODO else loguj, ze jedes na max, ale stejne malo
      }
      if(this->v_r < v_curr_r)
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
    if(this->v_l == 0 && abs(this->power_l-192) < 3)
    {
      this->power_l = 192;    
    }
    else
    {
      if(this->v_l > v_curr_l)
      {
        if(this->power_l < 255)
        {
          this->power_l++;
        }
        //TODO else loguj, ze jedes na max, ale stejne malo
      }
      if(this->v_l < v_curr_l)
      {
        if(this->power_l > 128)
        {
          this->power_l--;
        }
        //todo log, ze pomaleji to uz nepujde
      }

    }
    //TODO: pokud v_l/v_r jsou nulové a powers v intervalu netoceni, nastavit je na zastaveni, jinak to přepaluje motory. 
      this->set_power('r');
      this->set_power('l'); //TODO
  
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
 steps_right = 0;
 steps_left = 0;
}
