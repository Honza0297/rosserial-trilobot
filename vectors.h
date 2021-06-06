 /************************************************      */
 /*  Educational tutorial for Arduino in robotics       */
 /*  Vyukovy Tutorial pro pouziti Arduina v robotice    */
 /*  File: vectors.h                                    */
 /*  Author: Pololu & Jan Beran                         */
 /*  Date: March 2020                                   */
 /*                                                     */
 /* This file is a part of author´s bachelor thesis     */
 /*                                                     */ 
 /* Funkce jsou prevzaty nebo jsou inspirovany funkcemi z */
 /* https://github.com/pololu/minimu-9-ahrs-arduino     */
 /*******************************************************/

#ifndef _DATATYPES_H
#define _DATATYPES_H 1

/**
 * Vzor pro vektor
 * */
template <typename T> struct vector
{
    T x, y, z;
};

/**
 * @brief Vektorovy soucin
 * */
template <typename Ta, typename Tb> vector<float> vector_cross(const vector<Ta> a, const vector<Tb> b)
{
  vector<float> out = {0,0,0};
  
  out.x = (a.y * b.z) - (a.z * b.y);
  out.y = (a.z * b.x) - (a.x * b.z);
  out.z = (a.x * b.y) - (a.y * b.x);

  return out;
}

/**
 * @brief Skalarni soucin
 * */
template <typename Ta, typename Tb> float vector_dot(const vector<Ta> a, const vector<Tb> b)
{
  return (double)((a.x * b.x) + (a.y * b.y) + (a.z * b.z));
}

/**
 * @brief Absolutni hodnota z vektoru
 * */
template <typename T> float vector_abs(const vector<T> a)
{
  return sqrt(pow(a.x,2) + pow(a.y,2) + pow(a.z,2));
}

/**
 * @brief Normalizace vektoru.
 * */
template <typename T> vector<float> vector_normalize(vector<T> a)
{
  float mag = vector_abs(a);
  vector<float> ret = {0,0,0};
  ret.x = (float) a.x / mag;
  ret.y = (float) a.y / mag;
  ret.z = (float) a.z / mag;
  return ret;
}
#endif