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

#define Kwheel 0.0008377333 //Wheel constant -> pulses*Kwheel
#define Ks 0.8              //Sensor constant
#define rcm 0.0885          //Raio centro - motor
#define Dbar 0.098          //m
#define asd 0.25            //Angulo do sensor de dentro (rad)
#define asc 5.0             //Angulo do sensor do meio   (rad)
#define asf 10.0            //Angulo do sensor de fora   (rad)


//----------------------------------------------------------------------------------------------------------------
class MotorControl{
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
class Follow{
public:
    Follow(float kP, float kI, float kD, MotorControl *left, MotorControl *right, int loopTime);
    
    float getBattery(void);
    void waitButton(void);
    void Buzzer(bool);
    void start();
    void update(float linV);
    
    void updateSensor();
    void calcSensor();
    float getSensor();
    float getSensorSetpoint();
    bool getMark();
    
    float getDisplacement();
    float getLinearSpeed();
    float getAngularSpeed();
    float getRadius(); 
    
private:
    MotorControl *left_;
    MotorControl *right_;
    int time_;
    int loopTime_;
    float linV_;
    float angV_;
    float displacement_;
    float sensor_;
    
    float calcAngularSpeed();
    
    float kP_;
    float kI_;
    float kD_;
    
    float I;
    float error_;
};
//-------------------------------------------------------------------------------------------------------------
#endif