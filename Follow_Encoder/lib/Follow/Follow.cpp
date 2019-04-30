/*
*@Author:  Luciano Bonfim / Pedro Peres
*@Date:    16/04/2019 
*@Email:   lucianopbonfim@hotmail.com / phperesdasilva@gmail.com
*/
#include "Follow.h"

//------------------------------------------------------------------------------------------------------------------------------
//class Motor Control

MotorControl::MotorControl(float kP, float kI, float kD, TB6612 *motor, QEI *encoder)
            {
                
                //motor           (armazena endereço de leftPWM ou Right PWM)
                //*motor          (aponta para valor de leftPWM ou Right PWM)
                
                //motor_          (armazena valor de motor) que é o msm que (armazena endereço de leftPWM ou Right PWM)
                //*motor_         (aponta para valor de leftPWM ou Right PWM)
                
                motor_ = motor;
                encoder_ = encoder;
                
                kP_ = kP;
                kI_ = kI;
                kD_ = kD;
                
                Ds_ = 0;
                
                I=0;
                error_=0;
                displacement_ = 0;
                speed_ = 0;
                time_ = millis();    
            }
void MotorControl::update(float setpoint){
    
    int dt = millis() - time_;
    
    //Local variables for previous value
    float prevError = error_;
    float prevDisplacement = displacement_;
        
    //update motor displacement and speed
    displacement_ = (encoder_->getPulses())*Kwheel;
    Ds_ = (displacement_ - prevDisplacement); 
    
    speed_ = 1000*Ds_/dt;
        
    //update error and time
    time_ = millis();
    error_ =  setpoint - speed_; 
        
    //update PID
    float P = error_*kP_;
    I += (error_*(dt))*kI_;
    float D = ((error_-prevError)/(dt))*kD_;
        
    //Send the new PID value to the motor PWM
    motor_->speed(P+I+D);
    
}

float MotorControl::getDisplacement(){
    return (displacement_);
}

float MotorControl::getSpeed(){
    return (speed_);
}

float MotorControl::getDs(){
    return(Ds_);
    }

//------------------------------------------------------------------------------------------------------------------------------
//class Follow

//Sensors
AnalogIn SensorC(A1);
AnalogIn Sensor1(A2);
AnalogIn Sensor2(A3);
AnalogIn Sensor3(A4);
AnalogIn Sensor4(A6);
AnalogIn Sensor5(A0);
AnalogIn Sensor6(A5);

//Peripherals
DigitalOut Buzz(BUZZER_PIN);
DigitalIn Button(BUTTON_PIN);

//Battery
//AnalogIn Battery(Vbat);


Follow::Follow(float kP, float kI, float kD, MotorControl *left, MotorControl *right, int loopTime)
    {
        left_=left;
        right_=right;
        time_ = millis();
        loopTime_ = loopTime;
        linV_ = 0;
        angV_ = 0;  
        displacement_ = 0;
    
        sensor_ = 0; 
        
        kP_ = kP;
        kI_ = kI;
        kD_ = kD;
                
        I=0;
        error_=0;     

    }

void Follow::waitButton(){// wait until the button is pressed
     Button.mode(PullUp);
     while(1){
        if(!Button){break;}//Detect Button Press     
     }
}

void Follow::Buzzer(bool B){//(if B=1 buzzer on)(if B=0 buzzer off)
    Buzz=B;
}

void Follow::start(){//wait for the button to be pressed and then start the timer
    waitButton();
    wait(1);
    millisStart();
}

void Follow::calcSensor(){
    int i,cont = 0;
    
    bool s[] = {(Sensor1.read()<Ks),(Sensor2.read()<Ks),(Sensor3.read()<Ks),(Sensor4.read()<Ks),(Sensor5.read()<Ks),(Sensor6.read()<Ks)};
    cont = 0;
    for(i=0;i<6;i++){
        cont += s[i];   
    }

    if(cont!=0){
        sensor_=(s[0]*asf+s[1]*asc+s[2]*asd+s[3]*-asd+s[4]*-asc+s[5]*-asf)/cont;
    }
}

float Follow::getSensorSetpoint(){
    return((sensor_*linV_)/(Dbar*cos(sensor_)));
    }

float Follow::getSensor(){
    return(sensor_);
    }

bool Follow::getMark(){
    return (SensorC.read()<Ks);
}

float Follow::getDisplacement(){
    return (displacement_);
}
float Follow::getLinearSpeed(){
    return (linV_);
}

float Follow::getAngularSpeed(){ 
    return (angV_);    
}
float Follow::getRadius(){
    return 3;
}

    
void Follow::update(float setlinV){
    
    int dt = millis() - time_;
    
    if (dt>=loopTime_){
         
        time_ = millis();

        float Dds = (left_->getDs()-right_->getDs());           //updates delta displacement between wheels
        angV_ = (Dds/dt)/rcm;    
        displacement_ = (left_->getDisplacement() + right_->getDisplacement())/2;  
        linV_ = setlinV;//(left_->getDs() + right_->getDs())/(2*dt);      //updates linear velocity
        
        this -> calcSensor();
        
        //update error and prevError
        float prevError = error_;
        error_ =  getSensor(); 
        
        //update PID
        float P = error_*kP_;
        I += error_*kI_;
        float D = ((error_-prevError)/(dt))*kD_;
    
        left_->update(setlinV + (P+I+D) );//updates setpoint speed
        right_->update(setlinV - (P+I+D) );//updates setpoint speed
    
    }
}



