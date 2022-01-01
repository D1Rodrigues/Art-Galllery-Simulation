/*
 * @Description: CPS607-FinalExam, 
 * @Author: Dylan Rodrigues
 * @Date: Dec. 15, 2021
*/

#ifndef _Setup_H_
#define _Setup_H_

#include <arduino.h>
#include <Wire.h>
#include "I2Cdev.h"

#define PIN_Motor_STBY 4
#define PIN_Motor_PWMA 9
#define PIN_Motor_PWMB 6
#define PIN_Motor_AIN 8
#define PIN_Motor_BIN 7

#define speed_Max 255
#define normal_speed 170


/* initializing the functions I use */
class BotFunctionSet
{
  public:
  void BotMovement_Init(void); //initializing the pins
  /*following methods are for the OwlBot's movement*/
  void Move_Forward(uint8_t speed); 
  void Move_Backward(uint8_t speed);
  void Stop_Moving(void);
  void Turn_Right(void);
  void Turn_Left(void);
  void Stop_Gradually(void);
  
   /* initialize functions and variable dealing with the sensors */
  void Sensors_Init(void); //initializing  ultrasonic sensors
  #define IRSensor_Address 0XA0
  #define ULTRASONIC_Address 0x07
  bool Line_Checking(void);
  bool Obstacle_Avoidance(void);
  
  
  /* function and variable that deal with the joystick/rocker */
  void Rocker_Main(void);
  String CommandSerialNumber;
};

#endif
