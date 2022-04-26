 /************************************************ */
 /*  Educational tutorial for Arduino in robotics  */
 /*  File: srf08.h                                 */
 /*  Author: Jan Beran                             */
 /*  Date: autumn 2019                             */
 /* This file is a part of authors bachelor thesis */
 /*                                                */
 /* This file contains macros and function         */
 /* prototypes for controlling srf-08 ligh         */
 /* and distance sensor.                           */
 /**************************************************/
 
 #ifndef _SRF08_H
 #define _SRF08_H       1

#include "topics.h"
#include <ros.h>
#include <trilobot/Sonar_data.h>
#include <std_msgs/Empty.h>

/*
* Vychozi I2C adresy
*/


#define SRF08_ADDRESS_FRONT 0x74
#define SRF08_ADDRESS_FRONT_LEFT 0x71
#define SRF08_ADDRESS_FRONT_RIGHT 0x72
#define SRF08_ADDRESS_BACK_LEFT 0x75
#define SRF08_ADDRESS_BACK_RIGHT 0x70
#define SRF08_ADDRESS_BACK 0x73

#define SRF08_ADDRESS_BROADCAST 0x00
#define NUM_OF_SONARS 6

const uint8_t srf08_addresses[6] = {
  SRF08_ADDRESS_FRONT,
  SRF08_ADDRESS_FRONT_LEFT,
  SRF08_ADDRESS_FRONT_RIGHT,
  SRF08_ADDRESS_BACK_LEFT,
  SRF08_ADDRESS_BACK_RIGHT,
  SRF08_ADDRESS_BACK
};
/*
* Makra pro nastaveni vzdalenosti
*/
#define INCH 0x50
#define CM 0x51
#define MS 0x52
 
 /*
 * Makra pro pristup k nekterym registrum
 */ 
#define REG_CMD 0x00
#define REG_LIGH 0x01


#define SRF08_MEASURE_TIME 70
#define SRF08_UPDATE_INTERVAL 200-SRF08_MEASURE_TIME //[ms], substracting measure time to achieve desired period/frequency

typedef struct
{ 
  uint16_t front;
  uint16_t front_left;
  uint16_t front_right;
  uint16_t back_right;
  uint16_t back_left;
  uint16_t back;
} sonar_data;



/**
 * @brief Trida pro ovladani ultrazvukoveho sonaru SRF-08
 * */
class Sonar
{
    public:
        /**
         * @brief Konstruktor. 
         * */
        Sonar();
        /**
        * @brief Funkce ziska vzdalenost ze vsech senzoru SRF-08
        * @return Vzdalenost v pozadovane jednotce nebo -1 pri problemu
        */
        sonar_data get_distances(byte unit = CM);
        /**
        * @brief Funkce nastavuje mereni pomoci zapisu pozdadovane jednotky do REG_CMD.
        */
        void set_measurement(byte unit = CM, uint8_t address = SRF08_ADDRESS_BROADCAST);
};
class Sonar_driver
{
  private:
    ros::NodeHandle *nh;
    Sonar* sonar;
    unsigned long measure_start;
    unsigned long last_updated;
    bool measuring;
    ros::Publisher pub;
    trilobot::Sonar_data msg;
    void get_and_send_data();
    void callback(const std_msgs::Empty &msg);


  public:
     Sonar_driver(ros::NodeHandle *nh)
     : pub(topic_sonar_data, &msg)
     {
       this->nh = nh;
       this->sonar = new Sonar();
       this->measure_start = 0;
       this->last_updated = 0;
       this->measuring = false;

       this->nh->advertise(this->pub);
     };
     void update();

};
#endif
