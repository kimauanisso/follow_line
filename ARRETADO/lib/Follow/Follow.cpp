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
    displacement_ = getDisplacement();
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

void MotorControl::reset()
{
    encoder_->reset();
}

void MotorControl::stop()
{
    motor_->speed(0);
}

float MotorControl::getDisplacement()
{
    return (encoder_->getPulses() * Kwheel);
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
    mapLeft[100];//left wheel displacement
    mapRight[100];//right wheel displacement 
    mapLenght[100];//line lenght (can be curve or straight line)
    mapRadius[100];//line radius (can be curve or straight line) ->99999 for straight line
    speed[100];//speed
    markCount = 0;//number of marks read by the robt 
    //----------------------------------------------------------------------------------------------------------------------------------------------------

    left_ = left;           //left motor
    right_ = right;         //right motor
    time_ = millis();
    loopTime_ = loopTime;
    displacement_ = 0;     //displacement
    sensor_ = 0;           //line's position measured by the sensor bar

    //PID constants, error and I
    kP_ = kP;       
    kI_ = kI;
    kD_ = kD;
    I = 0;
    error_ = 0;

    Mark_Debouncing_Clock = 0;  //clock used for the mark sensor debouncing (starts at 0)

    mapLeft[0] = 0;//left wheel displacement
    mapRight[0] = 0;//right wheel displacement 

    fastLapCount = 0;//used on the fast lap
}

void Follow::waitButton()// wait until the button is pressed
{ 
    Button.mode(PullUp);
    while (1)
    {
        if (!Button)//Detect Button Press
        {
            break;
        } 
        wait(0.001);
    }
}

void Follow::start() //wait until the button is pressed, then reset the encoders and start the timer
{
    waitButton();

    Buzz = 1;
    wait(0.5);
    Buzz = 0;

    left_->reset();
    right_->reset();
    millisStart();
}

bool Follow::getMark() //return 1 if the robot reads a mark
{
    if (SensorC.read() < Ks)
    {
        if ((millis() - Mark_Debouncing_Clock) > MarkDebouncingTime)
        {
            Mark_Debouncing_Clock = millis();
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

float Follow::getSensor() //returns the SensorBar position
{
    return (sensor_);
    //return ((sensor_ * linV_) / (Dbar * cos(sensor_)));
}

float Follow::getDisplacement() //returns encoder measured displacement UNTIL that moment
{
    return ((left_->getDisplacement() + right_->getDisplacement()) / 2);
}

void Follow::bluetooth(){
    Serial bl(BL_RX,BL_TX);//turn Serial on
    waitButton();//wait for the button

    int i = 0;
    for(i=0; i<markCount; i++){
        bl.printf("%i--Lenght[m x 10^4]: %i--Radius[m x 10^4]: %i\n", i, int(mapLenght[i]*10000), int(mapRadius[i]*10000) );
    }
}

void Follow::stop(){//stop motors
    left_->stop();
    right_->stop();
}

void Follow::updateMapLap(float setlinV)//setpoint linear velocity
{

    if(getMark()){
        markCount++;
        mapLeft[markCount] = left_->getDisplacement();
        mapRight[markCount] = right_->getDisplacement();
    }
    
    int dt = millis() - time_;
    if (dt >= loopTime_)//if dt is bigger than looptime, updte de bar PID and also de motor´s PID 
    {
        time_ = millis();

        //update displacement
        displacement_ = this->getDisplacement();
        
        //update bar sensors
        this->calcSensor();

        //update error and prevError
        float prevError = error_;
        error_ = getSensor();

        //update PID
        float P = error_ * kP_;
        I += error_ * kI_;
        float D = ((error_ - prevError) / (dt)) * kD_;

        //update setpoint speed
        left_->update(setlinV + (P + I + D));
        right_->update(setlinV - (P + I + D));
    }
}

void Follow::Map()
{
    float angle;

    int i;
    for(i=0; i<markCount; i++){
        
        //lenght = displacement - last displacement
        mapLenght[i]=( (mapLeft[i+1]+mapRight[i+1])/2 ) - ( (mapLeft[i]+mapRight[i])/2 );

        //robot angle = (Dleft - Dright)Robot radius 
        angle = ( (mapLeft[i+1]-mapLeft[i]) - (mapRight[i+1]-mapRight[i]) ) / RobotRadius;        
        
        //curve
        if (angle>0.000001){
            //radius = arcLenght/angle
            mapRadius[i] = mapLenght[i] / angle;
        }
        //straight line
        else{
            mapRadius[i] = 99999;
        }
    }

    for(i=1; i<markCount; i++){
        mapLenght[i]=mapLenght[i]+mapLenght[i-1];
    }

}

float Follow::accelerationZone(float v0, float v1, float acceleration){//calculates the distance that the robot will need to start accelerating to achieve the next speed
    return mapLenght[fastLapCount] - (pow(v1, 2)-pow(v0,2))/(2*acceleration);
}

float Follow::calcSpeed(){   
    if( (this->getDisplacement()) > mapLenght[fastLapCount]){
        fastLapCount++;
    }
    return speed[fastLapCount];
}


void Follow::updateFastLap(float a, float maxSpeed)//setpoint acceleration, setpoint maxSpeed
{

    float setlinV = calcSpeed();

    int dt = millis() - time_;
    if (dt >= loopTime_)//if dt is bigger than looptime, update bar PID and also motor's PID 
    {
        time_ = millis();

        //update displacement
        displacement_ = this->getDisplacement();
        
        //update bar sensors
        this->calcSensor();

        //update error and prevError
        float prevError = error_;
        error_ = getSensor();

        //update PID
        float P = error_ * kP_;
        I += error_ * kI_;
        float D = ((error_ - prevError) / (dt)) * kD_;

        //update setpoint speed
        left_->update(setlinV + (P + I + D));
        right_->update(setlinV - (P + I + D));
    }
}