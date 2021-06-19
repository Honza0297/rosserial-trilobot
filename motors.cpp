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
  
  Serial1.begin(MOTOR_BAUDRATE);
  
  this->stop();
  attach_interrupts();
  //this->move_in_progress = false;
  //this->steps_to_go = -1; 
  
}
void Motors::set_power(char motor, byte power)
{
  switch (motor)
  {
    case 'r':
      this->power_r = power;
      Serial1.write(this->power_r);
      break;
    case 'l':
      this->power_l = power;
      Serial1.write(this->power_l);
      break;
    default:
    {}
  }
  return;  
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
