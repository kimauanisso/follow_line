/*
*@Author:  Luciano Bonfim / Pedro Peres
*@Date:    16/04/2019 
*@Email:   lucianopbonfim@hotmail.com / phperesdasilva@gmail.com
*/
#include "Follow.h"

//Sensors
AnalogIn SensorC(STURN);
AnalogIn Sensor1(S1);
AnalogIn Sensor2(S2);
AnalogIn Sensor3(S3);
AnalogIn Sensor4(S4);
AnalogIn Sensor5(S5);
AnalogIn Sensor6(S6);

//Peripherals
DigitalOut Buzz(BUZZER_PIN);
DigitalIn Button(BUTTON_PIN);
//AnalogIn Battery(Vbat);

//------------------------------------------------------------------------------------------------------------------------------
//class Motor Control

MotorControl::MotorControl(float kP, float kI, float kD, TB6612 *motor, QEI *encoder)
{
    motor_ = motor;
    encoder_ = encoder;

    kP_ = kP;
    kI_ = kI;
    kD_ = kD;

    Ds_ = 0;

    I = 0;
    error_ = 0;
    displacement_ = 0;
    speed_ = 0;
    time_ = millis();
}
void MotorControl::update(float setpoint)
{
    //delta time
    int dt = millis() - time_;

    //Local variables for previous value
    float prevError = error_;
    float prevDisplacement = displacement_;

    //update motor displacement and speed
    displacement_ = (encoder_->getPulses()) * Kwheel;
    Ds_ = (displacement_ - prevDisplacement);

    speed_ = 1000 * Ds_ / dt;

    //update error and time
    time_ = millis();
    error_ = setpoint - speed_;

    //update PID
    float P = error_ * kP_;
    I += (error_ * (dt)) * kI_;
    float D = ((error_ - prevError) / (dt)) * kD_;

    //Send the new PID value to the motor PWM
    motor_->speed(P + I + D);
}

float MotorControl::getDisplacement()
{
    return (displacement_);
}

float MotorControl::getSpeed()
{
    return (speed_);
}

float MotorControl::getDs()
{
    return (Ds_);
}

//------------------------------------------------------------------------------------------------------------------------------
//class Follow
Follow::Follow(float kP, float kI, float kD, MotorControl *left, MotorControl *right, int loopTime)
{
    //These Arrays will be calculated by the robot on its Mapping lap
    //----------------------------------------------------------------------------------------------------------------------------------------------------
    Map[100];//Mark position[m]
    MapRadius[100];//Curve Radius in [m], (straight line=9999999)
    automaticSpeed[100];//setpoint for linear speed [m/s]
    //----------------------------------------------------------------------------------------------------------------------------------------------------

    left_ = left;
    right_ = right;
    time_ = millis();
    loopTime_ = loopTime;
    linV_ = 0;
    angV_ = 0;
    displacement_ = 0;
    sensor_ = 0;

    kP_ = kP;
    kI_ = kI;
    kD_ = kD;
    I = 0;
    error_ = 0;

    Mark_Debouncing_Clock = 0;
}

void Follow::waitButton()// wait until the button is pressed
{ 
    Button.mode(PullUp);
    while (1)
    {
        if (!Button)
        {
            break;
        } //Detect Button Press
        wait(0.001);
    }
}

void Follow::start() //wait until the button is pressed and then start the timer
{
    waitButton();

    Buzz = 1;
    wait(0.5);
    Buzz = 0;

    millisStart();
}

bool Follow::getMark() //return 1 if the robot reads a mark
{
    if (SensorC.read() < Ks)
    {
        if ((millis() - Mark_Debouncing_Clock) > MarkDebouncingTime)
        {
            Mark_Debouncing_Clock = millis();
            Buzz= !Buzz;
            return 1;
        }
    }
    return 0;
}

void Follow::calcSensor()
{
    int cont = 0;
    int i = 0;

    bool s[] = {(Sensor1.read() < Ks), (Sensor2.read() < Ks), (Sensor3.read() < Ks), (Sensor4.read() < Ks), (Sensor5.read() < Ks), (Sensor6.read() < Ks)};
    
    for (i = 0; i < 6; i++)
    {
        cont += s[i];
    }

    if (cont != 0)
    {
        sensor_ = (s[0] * SEN2 + s[1] * SEN1 + s[2] * SEN0 + s[3] * -SEN0 + s[4] * -SEN1 + s[5] * -SEN2) / cont;
    }
}

float Follow::getSensorSetpoint()
{
    return ((sensor_ * linV_) / (Dbar * cos(sensor_)));
}

float Follow::getSensor() //returns the SensorBar position
{
    return (sensor_);
}

float Follow::getDisplacement() //returns encoder measured displacement UNTIL that moment
{
    return (displacement_);
}

float Follow::getLinearSpeed() //returns encoder measured linear velocity AT that moment
{
    return (linV_);
}

float Follow::getAngularSpeed() //returns encoder measured angular velocity AT that moment
{
    return (angV_);
}
float Follow::getRadius() //returns the Curve Radius
{
    float radius;  
    if(Ddisplacement_>0.00001){
        radius = ((displacement_*RobotRadius)/Ddisplacement_);
        Ddisplacement_=0;
        return radius;
    }
    else{
        return 999999;
    }

}

void Follow::bluetooth(){
    Serial bl(BL_RX,BL_TX);//ativa a serial
    int i = 0;
    waitButton();
    for(i=0; i<100; i++){
        if(automaticSpeed[i]!=0){
            bl.printf("Position:%i----Radius:%i----Speed:%i---[x10000]\n",int(Map[i]*10000), int(MapRadius[i]*10000), int(automaticSpeed[i]*10000));
        }
    }
}

void Follow::update(float setlinV)
{

    int dt = millis() - time_;

    if (dt >= loopTime_)
    {

        time_ = millis();

        displacement_ = (left_->getDisplacement() + right_->getDisplacement()) / 2;
        Ddisplacement_ += (left_->getDs() - right_->getDs());
        angV_ = ((left_->getDs() - right_->getDs()) / dt) / RobotRadius; //updates encoder measured angular velocity
        linV_ = (left_->getDs() + right_->getDs()) / (2 * dt);   //updates encoder measured linear velocity

        this->calcSensor();

        //update error and prevError
        float prevError = error_;
        error_ = getSensor();

        //update PID
        float P = error_ * kP_;
        I += error_ * kI_;
        float D = ((error_ - prevError) / (dt)) * kD_;

        //updates setpoint speed
        left_->update(setlinV + (P + I + D));
        right_->update(setlinV - (P + I + D));
    }
}