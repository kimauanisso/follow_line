/*
*@Author:  Luciano Bonfim
*@Date:    29/11/2018 
*@Email:   lucianopbonfim@hotmail.com
*/

#ifndef FOLLOW_H
#define FOLLOW_H

#include "QEI.h"
#include "TB6612.h"
#include "millis.h"
#include "pinup.h"

#define MarkDebouncingTime 200 
#define Kwheel 0.0008377333     //Wheel constant -> pulses*Kwheel
#define Ks 0.8                  //Sensor constant
#define RobotRadius 0.0885              //Raio centro - motor
#define Dbar 0.098              //m
#define SEN0 0.25                //Angulo do sensor de dentro (rad)
#define SEN1 5.0                 //Angulo do sensor do meio   (rad)
#define SEN2 10.0                //Angulo do sensor de fora   (rad)

//----------------------------------------------------------------------------------------------------------------
class MotorControl
{
  public:
    MotorControl(float kP, float kI, float kD, TB6612 *motor, QEI *encoder); //kP, kI, kD, looptime, leftPWM  address, leftEncoder  address
    void update(float setpoint);
    float getDisplacement();
    float getSpeed();
    float getDs();

    float displacement_;
    float speed_;

  private:
    float kP_;
    float kI_;
    float kD_;
    float I;
    float error_;

    float Ds_;

    TB6612 *motor_;
    QEI *encoder_;

    int time_;
};

//------------------------------------------------------------------------------------------------------------
class Follow
{
  public:
    Follow(float kP, float kI, float kD, MotorControl *left, MotorControl *right, int loopTime);

    float getBattery(void);
    void waitButton(void);
    void start();
    void update(float);

    void updateSensor();
    void calcSensor();
    float getSensor();
    float getSensorSetpoint();
    bool getMark();

    float getDisplacement();
    float getLinearSpeed();
    float getAngularSpeed();
    float getRadius();
    void bluetooth();
  
  //These Arrays will be calculated by the robot on its Mapping lap
  //----------------------------------------------------------------------------------------------------------------------------------------------------
  float Map[100];//Mark position[m]
  float MapRadius[100];//Curve Radius in [m], (straight line=9999999)
  float automaticSpeed[100];//setpoint for linear speed [m/s]
  //----------------------------------------------------------------------------------------------------------------------------------------------------

  private:
    MotorControl *left_;
    MotorControl *right_;

    int time_;
    int loopTime_;
    float linV_;
    float angV_;
    float displacement_;
    float Ddisplacement_;
    float sensor_;

    float calcAngularSpeed();

    float kP_;
    float kI_;
    float kD_;
    float I;
    float error_;

    int Mark_Debouncing_Clock;
};
//-------------------------------------------------------------------------------------------------------------
#endif