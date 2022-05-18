/************************************************ 
/*  Educational tutorial for Arduino in robotics  
/*    AND
/*  Docking Station for Automatic Charging of Batteries of Robot
/*  File: accelerometer.h                         
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
/**************************************************/
#ifndef _ACCELEROMETER_H
#define _ACCELEROMETER_H       1

#include <Arduino.h>
#include "vectors.h"

/* Adresa akcelerometru*/
#define ACCEL_ADDRESS 0x1d

/* Zacatek dat z akcelerometru*/
#define OUT_X_L_A 0x28

/* Konverze ze surovych dat do m/s**2 pri default hodnotach*/
#define ACCEL_CONVERSION_RATE 0.061/1000. 


/* @class Trida pro ovladani akcelerometru */
class Accelerometer
{
    public:
        /**
         * @brief Konstruktor. Nastavi zakladni hodnoty do regitru a jisti bias.
         * 
         * @param address Adresa zarizeni. Vychozi hodnota je nastavena na makro ACCELL_ADDRESS. 
         * */ 
        Accelerometer(byte address = ACCEL_ADDRESS);
        /**
         * @brief Funkce vraci surova data primo vyctena z registru akcelerometru.
         * */
        vector<int16_t> get_raw_data();
        /**
         * @brief Funkce vrai konvertovana data do m/s**2
         * */
        vector<float> get_acceleration();
        /**
         * @brief Funkce zjistuje, zda nedoslo k narazu. Dokud k narazu nedoslo, vraci false,
         * jakmile k narazu dojde, vraci true az do resetu funkce nebo MCU. 
         * @param treshold hranice narazu
         * @param reset Pokud je true, resetuje pamatovani narazu
         * 
         * @return Zda doslo k narazu
         * */
        bool check_impact(float treshold = 1.2, bool reset = false);
        /**
         * @brief Funkce pro doplneni behem tutorialu
         * */           
        bool detektor_narazu(float treshold, bool reset);
    private:
        /**
         * @brief Funkce zjisti odchylky mereni od nuloveho stavu. 
         * @note Pri nejim provadeni je vyzadovano, aby se s robotem nepohybovalo.
         * */
        void set_bias();
        /* Hodnoty biasu */
        vector<float> accel_bias;
        /* Adresa akcelerometru */
        byte accel_address;
};

 #endif 
