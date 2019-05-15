/*
*@Author:  Luciano Bonfim / Pedro Peres
*@Date:    16/04/2019 
*@Email:   lucianopbonfim@hotmail.com / phperesdasilva@gmail.com
*/
#include "Follow.h"

//Sensors
AnalogIn Sensor1(S1);
AnalogIn Sensor2(S2);
AnalogIn Sensor3(S3);
AnalogIn Sensor4(S4);
AnalogIn Sensor5(S5);
AnalogIn Sensor6(S6);
InterruptIn markSensor(STURN);

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
    speed[100];
    breakLenght[100];
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

    mapLeft[0] = 0;//left wheel displacement
    mapRight[0] = 0;//right wheel displacement 
    mapLenght[0] = 0;
    mapRadius[0] = 0;

    fastLapCount = 0;//used on the fast lap

    mark; 
    markCount = 0;//number of marks read by the robot

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

void Follow::start(){ //wait until the button is pressed, then reset the encoders and start the timer
    waitButton();

    Buzz = 1;
    wait(0.5);
    Buzz = 0;

    left_->reset();
    right_->reset();
    
    markSensor.rise(this, &Follow::getMark);
    millisStart();
}


void Follow::bluetooth(){
    Serial bl(BL_RX,BL_TX);//turn Serial on
    waitButton();//wait for the button

    int i = 0;
    for(i=0; i<=markCount; i++){
        bl.printf("%i --Lenght[mx10^4]: %i --Radius[mx10^4]: %i --speed[m/sx10^4]: %i \n", i, int(mapLenght[i]*10000), int(mapRadius[i]*10000), int(speed[i]*10000));
    }
}

void Follow::stop(){//stop motors
    left_->stop();
    right_->stop();
}

void Follow::getMark(){ //return 1 if the robot reads a mark
    mark = 1;
}

void Follow::calcSensor(){
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

float Follow::getSensor(){ //returns the SensorBar position
    return (sensor_);//return ((sensor_ * linV_) / (Dbar * cos(sensor_)));
}

float Follow::getDisplacement(){//returns encoder measured displacement UNTIL that moment
    return ((left_->getDisplacement() + right_->getDisplacement()) / 2);
}

void Follow::PID(float setlinV){//setpoint acceleration, setpoint maxSpeed
   
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

void Follow::Map(){
    
    markCount++;
    mapLeft[markCount] = left_->getDisplacement();
    mapRight[markCount] = right_->getDisplacement();
    
    float angle;
    int i;

    for(i=1; i<=markCount; i++){
    
        //lenght = displacement - last displacement
        mapLenght[i] = ( (mapLeft[i]+mapRight[i])/2 ) - ( (mapLeft[i-1]+mapRight[i-1])/2 );
        
        //robot angle = (Dleft - Dright)Robot radius 
        angle = ( (mapLeft[i]-mapLeft[i-1]) - (mapRight[i]-mapRight[i-1]) ) / (2*RobotRadius);        
        
        //curve
        if (fabs(angle)>0.0001){
            //radius = arcLenght/angle
            mapRadius[i] = mapLenght[i] / angle;
        }
        //straight line
        else{
            mapRadius[i] = 99999;
        }
    }

    for(i=1; i<markCount; i++){
        if( (fabs(mapRadius[i])>0.5) and (fabs(mapRadius[i+1])>0.5) )
        {
            mapLenght[i] = mapLenght[i] + mapLenght[i+1];
            int j;
            for (j = (i+1); j < markCount; j++)
            {
                mapLenght[j] = mapLenght[j+1];
                mapRadius[j] = mapRadius[j+1]; 
            }
            markCount--;
            i--;
       }
    }

    for(i=1; i<markCount; i++){
        if(fabs(mapRadius[i])>0.5)
        {
            speed[i] = MAXSPEED;
        }
        else{
            speed[i] = 0.3;
        }
    }


    for(i=1; i<=markCount; i++){
        mapLenght[i]=mapLenght[i]+mapLenght[i-1];
    }

    for(i=0; i<markCount; i++){//straight line break, if(straight line and if it is bigger than the breaking zone)
        if( ( fabs(mapRadius[i]) > 0.5 ) and ( (mapLenght[i]-mapLenght[i-1])>fabs(accelerationZone(speed[i-1],speed[i]))) ){
            int j;
            for (j = markCount; j >= i; j--)
            {
                speed[j+1] = speed[j];
                mapLenght[j+1] = mapLenght[j];
                mapRadius[j+1] = mapRadius[j];
            }
            speed[i+1] = speed [i+2];
            mapLenght[i] = mapLenght[i] - fabs(accelerationZone(speed[i],speed[i+1]) );
            markCount++;
       }
    }
}

float Follow::accelerationZone(float v0, float v1){//calculates the distance that the robot will need to start accelerating to achieve the next speed
    return ( (pow(v1, 2) - pow(v0,2)) /(2*ACCELERATION) );//torricelli
}

float Follow::accelerate(float ds, float v0){//calculates the distance that the robot will need to start accelerating to achieve the next speed
    return sqrt( fabs(pow(v0,2) + 2*ACCELERATION*ds) );//torricelli
}

void Follow::updateMapLap(float setlinV){//setpoint linear velocity

    if(mark){
        //Buzz=!Buzz;
        markCount++;
        mapLeft[markCount] = left_->getDisplacement();
        mapRight[markCount] = right_->getDisplacement();
        mark=0;
    }
    
    PID(setlinV);
}

void Follow::updateFastLap(float acceleration, float maxSpeed){//setpoint acceleration, setpoint maxSpeed

    if( (this->getDisplacement()) >= mapLenght[fastLapCount]){
        fastLapCount++;
    }

    float linearVelocity = speed[fastLapCount];
    float ds;

    if( fabs(mapRadius[fastLapCount])>2 ){
        ds = (this->getDisplacement()) - mapLenght[fastLapCount-1];
        
        if(speed[fastLapCount]<speed[fastLapCount-1]){
            ds = -ds;
        }

        linearVelocity = accelerate(ds, speed[fastLapCount-1]);
        
        if(linearVelocity>speed[fastLapCount] and ds>0){
            linearVelocity=speed[fastLapCount];
        }

    }

    PID(linearVelocity);
}