 /************************************************ */
 /*  Educational tutorial for Arduino in robotics  */
 /*  Vyukovy Tutorial pro pouziti Arduina v robotice*/
 /*  File: motors.h                                */
 /*  Author: Jan Beran                             */
 /*  Date: autumn 2019                             */
 /*                                                */
 /* This file is a part of author´s bachelor thesis*/
 /*                                                */
 /**************************************************/
 #ifndef _MOTORS_H
 #define _MOTORS_H       1

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "topics.h"
#include <trilobot/Odometry.h>


/* Pocet kroku enkoderu pri vyuziti obou kanalu */
#define STEPS_AB 3072.0
/* Pocet kroku enkoderu pri pouziti jednoho kanalu */
#define STEPS_ONE_CHANNEL 768.0

/* Rychlost prenosu po seriove lince pro ovladani motoru */
#define MOTOR_BAUDRATE 9600

/* Zastavujici byte */
#define STOP_BYTE 0x00

/* Obvod hnanych kol robota */ // zmenena hodnota, puvodne to bylo v cm (=27.9)
#define WHEEL_CIRCUIT 0.279

/* Vzdálenost mezi koly, 20 cm */ 
#define INTERWHEEL_DISTANCE 0.2

/* Piny pro kanaly enkoderu */
#define LEFT_A 2
#define LEFT_B 53

#define RIGHT_A 3
#define RIGHT_B 52

#define TURN_RADIUS 40

/* minimalni rychlost, m/s*/
#define MIN_VELOCITY 0.02

#define POWER_STOP_L 192
#define POWER_STOP_R 64

enum State {move = 1, stop = 0}; 


/**
 * @brief Funkce pro ovladani preruseni
*/
void motor_right_interrupt_handler();
void motor_left_interrupt_handler();
void attach_interrupts();
void detach_interrupts();


/**
 * @brief Trida Motors slouzi pro ovladani motoru pomoci Sabertooth 2x5
*/
class Motors
{
    public:
        Motors();
        void set_power(byte l, byte r);
        void stop();  
        void update();
        int get_dir_coef(byte power);
        float vel_l;
        float vel_r;
        byte power_r;
        byte power_l;
        unsigned long start_time_r;
        unsigned long start_time_l; 
        bool moving();
        byte get_power(char motor);
};

template<typename MT> 
class Pub_wrapper
{
  public: 
    MT msg;
    char* topic;
    ros::Publisher pub;
    Pub_wrapper(char * top)
    : topic(top), pub(topic, &msg)
    {
    };
  
};

class Motor_driver
{
    private:
        ros::NodeHandle *nh;
        
        float desired_speed_l;
        unsigned long  timestamp_l;
        unsigned long last_ticks_l;

        float desired_speed_r;
        unsigned long  timestamp_r;
        unsigned long last_ticks_r;
        trilobot::Odometry msg;
        ros::Publisher pub;
        ros::Subscriber<geometry_msgs::Twist, Motor_driver> vel_sub;
        Motors *motors;
        //Pub_wrapper<trilobot::Odometry> pub;//(trilobot::Odometry);//(topic_trilobot_odometry);
        
        long last_update;
        int timeout;


        //void require_state(State state);

    public:
        Motor_driver(int timeout,ros::NodeHandle *nh)
        : pub(topic_trilobot_odometry, &msg), vel_sub(topic_cmd_vel, &Motor_driver::vel_callback, this)
        {
          this->nh = nh;
          this->timeout = timeout;
          this->motors = new Motors();
          this->set_desired_speed(0,0);
          this->timestamp_r = 0;
          this->timestamp_l = 0;
          //this->odometry_pub(topic_trilobot_odometry, &odometry_msg);
          this->nh->subscribe(this->vel_sub);
          this->nh->advertise(this->pub);
  
        };
        void update();
        void stop();
        void emergency_stop();
        void set_desired_speed(float l, float r);

        void vel_callback(const geometry_msgs::Twist &msg);

        //void get_desired_speed();
        //void get_real_speed();
};
 #endif
