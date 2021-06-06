 /************************************************ */
 /*  Educational tutorial for Arduino in robotics  */
 /*  Vyukovy Tutorial pro pouziti Arduina v robotice*/
 /*  File: magnetometer.cpp                        */
 /*  Author: Jan Beran                             */
 /*  Date: March 2020                              */
 /*                                                */
 /* This file is a part of author´s bachelor thesis*/
 /*                                                */
 /**************************************************/

#include "magnetometer.h"
#include "I2C.h"
#include "vectors.h"

Magnetometer::Magnetometer(Accelerometer *accel, byte address)
{   
  Wire.begin();
  this->mag_address = address;
  // 0x70 = 0b01110000
  // M_RES = 11 (high resolution mode); M_ODR = 100 (50 Hz ODR)
  reg_write(this->mag_address, 0x24, 0x70);

  // 0x20 = 0b00100000  
  // MFS = 01 (+/- 4 gauss full scale)
  reg_write(this->mag_address, 0x25, 0x20);

  // 0x00 = 0b00000000
  // MLP = 0 (low power mode off); MD = 00 (continuous-conversion mode)
  reg_write(this->mag_address, 0x26, 0x00);

  //TODO
  /*Az bude fungovat set limits, odkomentovat*/
  //this->set_limits();
  this->mag_min = {-190, -850, +3460};
  this->mag_max = {+2100, +1280, +3800};
  this->set_origin_angle(accel);
}
void Magnetometer::set_origin_angle(Accelerometer *accel)
{
  this->origin_from_N = heading(accel);
}


float Magnetometer::heading(Accelerometer *accel)
{
  /*Udava, jaky smer je "predek" (osa X - prvni pozice). Jelikoz je pro lepsi pristup senzor "opacne", 
  je hodnota zaporna. */
  vector<int> front = {-1,0,0};

  vector<int16_t> mag_values = this->get_intensity();
  vector<float> accel_values = accel->get_acceleration();

  // subtract offset (average of min and max) from magnetometer readings
  mag_values.x -= (mag_min.x + mag_max.x) / 2;
  mag_values.y -= (mag_min.y + mag_max.y) / 2;
  mag_values.z -= (mag_min.z + mag_max.z) / 2;

  // compute E and N
  vector<float> E;
  vector<float> N;
  E = vector_cross(mag_values, accel_values);
  E = vector_normalize(E);
  N = vector_cross(accel_values, E);
  N = vector_normalize(N);

  /* Vzorec pro uhel mezi dvema vektory*/
  float angle = acos(vector_dot(N, front)
                     /
                     (vector_abs(N)*vector_abs(front))
                    )* 180 / PI;
  
    return angle;

}

void Magnetometer::set_limits()
{
  // TODO az bude tlacitko, tak zacit po zmacknuti - pocita se s pohybem
  int cnt = 0;
  while(cnt > 50)
  {
    vector <int16_t> vals = this->get_intensity();
    mag_min.x = min(mag_min.x, vals.x);
    mag_min.y = min(mag_min.y, vals.y);
    mag_min.z = min(mag_min.z, vals.z);

    mag_max.x = max(mag_max.x, vals.x);
    mag_max.y = max(mag_max.y, vals.y);
    mag_max.z = max(mag_max.z, vals.z);
    delay(20);
  }
}

vector<int16_t> Magnetometer::get_intensity()
{
  vector<int16_t> return_vec;

  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write(OUT_X_L_M | (1 << 7)); // horni bit 1 a pak 7-bit adresa registru 0x28
  Wire.endTransmission();

  Wire.requestFrom(MAG_ADDRESS, 6);
  while(Wire.available() < 6);
  
  int8_t xl = Wire.read();
  int8_t xh = Wire.read();
  
  int8_t yl = Wire.read();
  int8_t yh = Wire.read();
  
  int8_t zl = Wire.read();
  int8_t zh = Wire.read();
   

  return_vec.x = (int16_t)((xh << 8) + xl);
  return_vec.y = (int16_t)((yh << 8) + yl);
  return_vec.z = (int16_t)((zh << 8) + zl);
  return return_vec;
}

float Magnetometer::heading_simple()
{
  vector<int16_t> mag_values = this->get_intensity();
   /*Udava, jaky smer je "predek" (osa X - prvni pozice). Jelikoz je pro lepsi pristup senzor "opacne", 
  je hodnota zaporna. */
  vector<int> front = {-1,0,0};

  mag_values.x -= (mag_min.x + mag_max.x) / 2;
  mag_values.y -= (mag_min.y + mag_max.y) / 2;
  mag_values.z = 0;
  vector<float> norm_mag_values = vector_normalize(mag_values);
    
  float angle = acos(vector_dot(norm_mag_values, front)
                    /
                    (vector_abs(norm_mag_values)*vector_abs(front))
                  )* 180 / PI;
  
  return angle;
}

float Magnetometer::kompas_1()
{
  vector<int16_t> mag_values = {0,0,0}/*TODO ziskat data intenzity z magnetometru*/;
   /*Udava, jaky smer je "predek" (osa X - prvni pozice). Jelikoz je pro lepsi pristup senzor "opacne", 
  je hodnota zaporna. */
  vector<int> front = {-1,0,0};

  mag_values.x -= (mag_min.x + mag_max.x) / 2;
  mag_values.y -= (mag_min.y + mag_max.y) / 2;
  /*TODO vynulovat souradnici Z*/
  vector<float> norm_mag_values = vector_normalize(mag_values);
    
  float angle = 0;/*TODO spocitat uhel mezi vektory dle navodu*/
  return angle;
}

float Magnetometer::kompas_2(Accelerometer *accel)
{  /*Udava, jaky smer je "predek" (osa X - prvni pozice).
   * Jelikoz je pro lepsi pristup senzor "opacne", 
   * je hodnota zaporna. */
  vector<int> front = {-1,0,0};
  vector<int16_t> mag_values = this->get_intensity();
  vector<float> accel_values = accel->get_acceleration();
  /** odecteni offsetu */
  mag_values.x -=(mag_min.x+mag_max.x)/2;
  mag_values.y -=(mag_min.y+mag_max.y)/2;
  mag_values.z -=(mag_min.z+mag_max.z)/2;
  
  vector<float> E; /* Vychod */
  vector<float> N; /* Sever*/
  /* TODO: spocitat sever podle navodu */

  float angle = 0; /*TODO uhel mezi vektory N a front*/
  return angle;
   }