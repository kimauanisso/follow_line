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

#define MarkDebouncingTime 100
#define Kwheel 0.0008377333     //Wheel constant -> pulses*Kwheel
#define Ks 0.8                  //Sensor constant
#define RobotRadius 0.0885              //Raio centro - motor
#define Dbar 0.098              //m
#define SEN0 0.15                //Angulo do sensor de dentro (rad)
#define SEN1 2.0                 //Angulo do sensor do meio   (rad)
#define SEN2 13.0                //Angulo do sensor de fora   (rad)

//----------------------------------------------------------------------------------------------------------------
class MotorControl
{
  public:
    MotorControl(float kP, float kI, float kD, TB6612 *motor, QEI *encoder); //kP, kI, kD, looptime, leftPWM  address, leftEncoder  address
    void update(float setpoint);
    float getDisplacement();
    float getSpeed();
    float getDs();
    void stop();
    void reset();

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
    void waitButton(void);//wait for the button
    void start();//starts clock when the button is pressed
    void updateMapLap(float);//update map lap with the linear speed[m/s] as parameter
    void updateFastLap(float, float);//update fast lap with the linear speed[m/s] as parameter

    void calcSensor();
    float getSensor();
    bool getMark();
    void Map();

    float getDisplacement();
    void bluetooth();
    void stop();
  
    //These Arrays will be calculated by the robot on its Mapping lap
    //----------------------------------------------------------------------------------------------------------------------------------------------------
    float mapLeft[100];//Mark position[m]
    float mapRight[100];
    float mapLenght[100];//line lenght (can be curve or straight line)
    float mapRadius[100];//line radius (can be curve or straight line) ->99999 for straight line
    int markCount;
    //----------------------------------------------------------------------------------------------------------------------------------------------------

  private:
    MotorControl *left_;
    MotorControl *right_;

    int time_;
    int loopTime_;
    float linV_;
    float displacement_;
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