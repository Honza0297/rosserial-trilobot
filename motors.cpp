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

  Serial1.begin(MOTOR_BAUDRATE);
  this->stop();
  steps_left = 0;
  steps_right = 0;
  this->move_in_progress = false;
  this->steps_to_go = -1; 
  
}

void Motors::stop_after_distance()
{
  this->stop();
  this->move_in_progress = false;
  detach_interrupts();
}

bool Motors::distance_reached()
{
  return (steps_left >= this->steps_to_go || steps_right >= this->steps_to_go);
}

void Motors::newmove(float l, float r)
{
  
  time_left = micros();
  time_right = micros();
  int perc_left = 10;
  int perc_right = 10;
  attach_interrupts();
  while(!(abs(speed_left-l) < delta) &&
        !(abs(speed_right-r) < delta))
        {
          if(speed_left < l)
          {
            perc_left++;
            byte lbyte = get_speed_from_percentage(perc_left);
            Serial1.write(lbyte+127);    
          }
          else
          {
            perc_left--;
            byte lbyte = get_speed_from_percentage(perc_left);
            Serial1.write(lbyte+127);
          }
          if(speed_right< l)
          {
            perc_right++;
            byte lbyte = get_speed_from_percentage(perc_right);
            Serial1.write(lbyte);    
          }
          else
          {
            perc_right--;
            byte lbyte = get_speed_from_percentage(perc_right);
            Serial1.write(lbyte);
          }
          speed_left = (WHEEL_CIRCUIT*steps_left/STEPS_ONE_CHANNEL)/(micros()-time_left)*1000000;
          speed_right = (WHEEL_CIRCUIT*steps_right/STEPS_ONE_CHANNEL)/(micros()-time_right)*1000000; 
        }
  detach_interrupts();
}

bool Motors::is_move_in_progress()
{
  return this->move_in_progress;
}

byte Motors::get_speed_from_percentage(int speed)
{
  /* Transformace rychlosti, aby robot jel na spravnou stranu*/
  speed *= -1;
  
  if(speed > 100 || speed < -100)
    return 0x00;

  byte control_byte = 0x00;
  
  /* Mezni hodnoty reseny explicitne. Zamezi se problemum
    typu round vs floor vs truncate*/
  switch(speed)
  {
    case -100:
      control_byte = 1;
      break;
    case 0:
      control_byte = 64;
      break;
    case 100:
      control_byte = 127;
      break;
    default:
      /* Normalizace do rozsahu 0-200 */
      speed += 100;
      /* truncate + 0.5 = round*/ 
      control_byte = (byte) (127./200*speed + 0.5); 
      break;
  }
  
  return control_byte;
}

void Motors::move(int distance, int speed)
{
  
  /*Pokud je vzdalenost zaporna, upravime do ocekavaneho vstupu*/
  if(distance < 0)
  {
    distance *= -1;
    speed *= -1;
  }
  this->steps_to_go = (int)(distance/WHEEL_CIRCUIT*STEPS_ONE_CHANNEL);
  attach_interrupts();
  this->move_in_progress = true;
  this->move(speed);
 /* while(1)
  {
    if(steps_left == steps_to_go || steps_right == steps_to_go) 
      break;
  }
  this->stop();
  detach_interrupts();*/
  return;

}

void Motors::move(int speed)
{
  byte control_byte = get_speed_from_percentage(speed);
  Serial.write("OK");
  Serial1.write(control_byte);
  Serial1.write(control_byte+127); 
}

void Motors::turn(int angle, int speed)
{
 
  bool right = true;
  if(angle > 0)
    right = true;
  else if (angle < 0)
    {
      right = false;
      angle *= -1;
    }
    
  int steps_to_go = (int) ((40 * PI * angle / 360) / WHEEL_CIRCUIT * STEPS_ONE_CHANNEL);
  
  attach_interrupts();
  
  if(right == true)
    this->turn_right(speed);
  else if (right == false)
    this->turn_left(speed);

  while(1)
  {
    if(steps_left == steps_to_go || steps_right == steps_to_go) 
      break;
  }
  this->stop();
  detach_interrupts();
}

void Motors::turn_left(int speed)
{
  byte control_byte = Motors::get_speed_from_percentage(speed);
  Serial1.write(control_byte);
}

void Motors::turn_right(int speed)
{
  byte control_byte = Motors::get_speed_from_percentage(speed);
  Serial1.write(control_byte+127);
}

void Motors::circle(int diameter, bool countercloockwise)
{
  /*TODO*/
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
  steps_right++;
}

/** @brief Funkce pro obsluhu preruseni z leveho motoru */
void motor_left_interrupt_handler()
{
  steps_left++;
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
