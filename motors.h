 /************************************************ 
 /*  Educational tutorial for Arduino in robotics  
 /*    AND
 /*  Docking Station for Automatic Charging of Batteries of Robot
 /*  File: motors.h                                
 /*  Author: Jan Beran                             
 /*  Date: autumn 2019 and 2020-2022               
 /*                                                
 /* This file is a part of author´s diploma thesis
 /* This file used parts of code from author's bachelor thesis 
 /*                                                
 /**************************************************/
 #ifndef _MOTORS_H
 #define _MOTORS_H       1

#include <ros.h>
#include <trilobot/Vel.h>
#include "topics.h"
#include <trilobot/Odom.h>

/* Encoder ticks per rotation in two-channel mode */
#define STEPS_AB 3072.0

/* Encoder ticks per rotation in one-channel mode */
#define STEPS_ONE_CHANNEL 768.0

#define MOTOR_BAUDRATE 9600

#define STOP_BYTE 0x00

#define WHEEL_CIRCUIT 0.279 // [ms]

#define INTERWHEEL_DISTANCE 0.2 // [m]

/* Encoder pins */
#define LEFT_A 2
#define LEFT_B 53

#define RIGHT_A 3
#define RIGHT_B 52

/* Minimal robot speed and minimal power to send in Sabertooth to ensure motor will spin */
#define MIN_VELOCITY 0.03
#define MIN_POWER 4


#define POWER_STOP_L 192
#define POWER_STOP_R 64

/* Direction mapping */
#define DIR_NONE 0
#define DIR_FORW 1
#define DIR_BACK 2

void motor_right_interrupt_handler();
void motor_left_interrupt_handler();
void attach_interrupts();
void detach_interrupts();

class Motors
{
    public:
        Motors();
        void set_power(byte l, byte r);
        void stop();  
        int get_dir_coef(byte power);
        float vel_l;
        float vel_r;
        byte power_r;
        byte power_l;
        unsigned long start_time_r;
        unsigned long start_time_l; 
        byte get_current_power(char motor);
};

class Motor_driver
{
    private:
        ros::NodeHandle *nh;
        
        float goal_speed_l;
        unsigned long  timestamp_l;
        unsigned long last_ticks_l;

        float goal_speed_r;
        unsigned long  timestamp_r;
        unsigned long last_ticks_r;

        trilobot::Odom msg;
        ros::Publisher pub;
        ros::Subscriber<trilobot::Vel, Motor_driver> vel_sub;
        Motors *motors;
        
        long last_update;
        int timeout;
        byte compute_new_power(char motor);


    public:
        Motor_driver(int timeout,ros::NodeHandle *nh)
        : pub(topic_trilobot_odometry, &msg),
          vel_sub(topic_cmd_vel, &Motor_driver::vel_callback, this)
        {
          this->nh = nh;
          this->timeout = timeout;
          this->motors = new Motors();
          this->set_goal_speed(0,0);
          this->timestamp_r = 0;
          this->timestamp_l = 0;
          this->nh->subscribe(this->vel_sub);
          this->nh->advertise(this->pub);  
          this->last_update = millis();
          this->goal_speed_l = 0;
          this->goal_speed_r = 0;
        };
        void update();
        void stop();
        void emergency_stop();
        void set_goal_speed(float l, float r);

        void vel_callback(const trilobot::Vel &msg);
};
 #endif
