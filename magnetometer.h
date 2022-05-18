 /************************************************ 
 /*  Educational tutorial for Arduino in robotics  
 /*    AND
 /*  Docking Station for Automatic Charging of Batteries of Robot
 /*  File: magnetometer.h                          
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

 
#ifndef _MAGNETOMETER_H
#define _MAGNETOMETER_H       1

#include <Arduino.h>
#include "vectors.h"
#include "accelerometer.h"

/* I2C adresa magnetometru (shodna s akcelerometrem) */
#define MAG_ADDRESS 0x1d

/* Zacatek vystupnich dat akcelerometru */
#define OUT_X_L_M 0x08

/**
 * @brief Trida pro ovladani magnetometru
*/
class Magnetometer
{
    public:
        /**
         * @brief Konstruktor. Nastavi vychozi hodnoty do registru magnetometru a urci vychozi uhel mezi predkem
         * robota a severem, kvuli cemuz potrebuje acelerometr
         * */
        Magnetometer(Accelerometer *accel, byte address = MAG_ADDRESS);
        /**
         * @brief Vrati hodnotu intenzity magnetickeho pole pro vsechny tri osy XYZ.
         * */
        vector<int16_t> get_intensity();
        /**
         * @brief Vrati uhel mezi predkem robota a severem.
         * */
        float heading(Accelerometer *accel);
        /**
         * @brief Zjednoduseni funkce heading - neni tolik presne.
         * 
         * @return Uhel mezi predkem robota a severem s presnosti kolem 20 stupnu.
         * @warning Funkce vraci pouze kladny uhel mezi 0 a 180 stupni.
         * */
        float heading_simple();
        /**
         * @brief Kostra pro doplneni behem tutorialu
         * @warning Funkce vraci pouze kladny uhel mezi 0 a 180 stupni.
         * 
         * @return Uhel mezi severem a predkem robota
         * */
        float kompas_1();
         /**
         * @brief Kostra pro doplneni behem tutorialu
         * @warning Funkce vraci pouze kladny uhel mezi 0 a 180 stupni.
         * 
         * @return Uhel mezi severem a predkem robota
         * */
        float kompas_2(Accelerometer *accel);
         /**
         * @brief Minimum hodnot magneticke intenzity
         * */
        vector<int16_t> mag_min;
        /** 
         * @brief Maximum hodnot magneticke intenzity.
         * */
        vector<int16_t> mag_max;
    private:
        /** 
         * @brief I2C adresa konkretni instance magnetometru.
         * */
        byte mag_address;
        /**
         * @brief Vychozi uhel mezi predkem robota a severem.
         * */
        float origin_from_N = 0;
        /**
         * @brief Nastavi nova minima a maxima hodnot intenzity. Je nutne, aby se s robotem
         * behem nastavovani hybalo!
         * */
        void set_limits();
        /**
         * @brief Nastavi vychozi uhel mezi severem a predkem robota. Volano v konstruktoru.
         * */
        void set_origin_angle(Accelerometer *accel);
};

 #endif