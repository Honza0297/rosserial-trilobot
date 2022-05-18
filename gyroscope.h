 /************************************************ 
 /*  Educational tutorial for Arduino in robotics  
 /*    AND
 /*  Docking Station for Automatic Charging of Batteries of Robot 
 /*  File: gyroscope.h                             
 /*  Author: Jan Beran                             
 /*  Date: March 2020                              
 /*                                                
 /* This file is a part of author´s diploma thesis
 /*                                                
 /* Justification to include it in the diploma thesis: 
 /* minIMUv3 is mounted on current Trilobots as well 
 /* and once will be integrated. 
 /* By now, this file WAS NOT CHANGED SINCE THE BACHELOR THESIS
 /* (excluding this header)
 /*                                                
 /**************************************************/

#ifndef _GYROSCOPE_H
#define _GYROSCOPE_H  1

#include <Arduino.h> 
#include "vectors.h"

/*Konverze ze surovych hodnot do stupnu za sekundu*/
#define GYRO_CONVERSION_RATE 8.75/1000.

/* I2C adresa gyroskopu */
#define GYRO_ADDRESS 0x6B

/***********************/
/* Popisy registru    */
/***********************/

/* Adresa who-a,-i registru */
#define WHO_AM_I   0x0F

/*
* |DR1|DR0; output data rate, |BW1|BW0|PD|Zen|Yen|Xen|
* DR = 01
* BW = 10
* PD = 1
* Zen,Xen,Yen = 111
* default = 0b01101111 (outpout rate = 200 Hz, bandwitch 50 Hz, power on, all axis)
*/
#define CTRL1  0x20

/* Filtering, external trigger enable...Default values are OK  */
#define CTRL2    0x21 

/* Interrupt settings, default values are OK*/
#define CTRL3    0x22 

/*|BDU; block data update; 0 = continual, 1 only after read|BLE; 0 = little endian, 1 = big endian|FS0|FS1; full scale selection, 0 default; 245 dps|
*       0|ST2|ST1 = self tests, default = 0|SPI interface mode|
*  default: 0b00000000 
*/
#define CTRL4   0x23 
       
/*
* |BOOT|FIFO_EN|stop_on_FTH||||||
* BOOT - reboot with 1 set here
* FIFO enable, 1 = enable, 0 disable (default)
* stop_on_fth = when FIFO treshold set, here it is enabled
* Default values are OK
*/
#define CTRL5    0x24

/*
* Angular velocity/rate as two´s complement for all axis.
* Every value is in two registers 
*/
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D

/* temperature data register, two´s complement*/
#define OUT_TEMP 0x26 

/*
* |-|-|DRDY_GL|0|I2C_disable|SW_RES|0|Low_ODR|
* DRDY_HL - interrupt2 pin active level, default 0
* I2C_dis: 1 = i2c disabled, 0 = i2c enabled
* SW_RES; software reset if 1
* Low_ODR - low output data rate, can be set by combination of Low_odr, DR and BW
*/
#define LOW_ODR 0x39  

/**
 * @brief Trida gyroskop reprezentujici gyroskop.
 * */
class Gyroscope
{
    public:
        /**
         * @brief Konstruktor. Nastavi zakladni hodnoty a zkalibruje senzor
         * */ 
        Gyroscope(byte address = GYRO_ADDRESS);
        /**
         * @brief Vrati hodnotu registru OUT_TEMP. Jelikoz neni teplota urcena pro realne pouziti, 
         * hodnoty mohou byt nepresne.  
         * */
        byte get_temperature();
        /**
         * @brief Funkce vraci surova data prectena z registru gyroskopu
         * */
        vector<int16_t> get_raw_data();
        /**
         * @brief Funkce vrati uhlovou rychlost pomoci konverze ze surovych dat z gyroskopu
         * */
        vector<float> get_angular_velocity();
        /**
         * @brief Funkce overi, zda neni s robotem nadmerne treseno.
         * @note Je doporuceno volat funkci primerene casto (nekolikrat za vterinu). 
         * 
         * @param treshold Parametr nastavuje citlivost funkce na otres
         * 
         * @return true, pokud je prekrocen treshold alespon na jedne ose, jinak false.
         * */
        bool check_shake(float treshold = 10);
         /**
         * @brief Funkce pro doplneni behem tutorialu
         * @note Je doporuceno volat funkci primerene casto (nekolikrat za vterinu). 
         * 
         * @param treshold Parametr nastavuje citlivost funkce na otres
         * 
         * @return true, pokud je prekrocen treshold alespon na jedne ose, jinak false.
         * */
        bool detektor_otresu(float treshold);
    private:
        /* Odchylky od nuly pro kazdou ze tri os */
        vector<float> BIAS;
        /* I2C adresa gyroskopu */
        int gyro_address; 
        /**
         * @brief Kalibracni funkce gyroskopu
         * */
        void get_bias(int num_of_measurements = 14);

};
#endif