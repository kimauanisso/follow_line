#include "Follow.h"
#include "mbed.h"


//Motor PWM
TB6612 leftPWM(PWMA, AIN1, AIN2);
TB6612 rightPWM(PWMB, BIN1, BIN2);

//Encoders
QEI leftEncoder(OUTA1, OUTB1, NC, 120, QEI::X4_ENCODING);  //chanA, chanB, index, ppr
QEI rightEncoder(OUTA2, OUTB2, NC, 120, QEI::X4_ENCODING); //chanA, chanB, index, ppr ->12 readings per revolution * Gear Ratio=1:10

//Motor Controlled Speed
MotorControl leftMotor(1.7, 0.000005, 0, &leftPWM, &leftEncoder);  //kP, kI, kD, looptime, leftPWM  address, leftEncoder  address
//                     3.2, 0.000005, 0 --> bat 8.00
MotorControl rightMotor(1.9, 0.000005, 0, &rightPWM, &rightEncoder); //kP, kI, kD, RightPWM address, RightEncoder address

//Follow Class----angular velocity, sensor bar, curve radius...
Follow follow(0.030, 0, 0.06 , &leftMotor, &rightMotor, 20);//kp, ki, kd, leftmotor(object), rightmotor(object), looptime
//            0.025, 0, 0.06 --> bat 8.00

float distance[100];
float velocity[100];
float count = 0;

int main()
{
    //wait(1);
    //follow.start();//BANANA
    
    //while (follow.getDisplacement()<16.5){
    //    follow.updateMapLap(0.3);
    //}
    
    //follow.stop();
    //follow.Map();
    //follow.bluetooth();

    wait(1);
    follow.start();

    while (follow.getDisplacement()<16.5){
        follow.updateFastLap(0.9);
    }
    while (follow.getDisplacement()<1.10){
        follow.updateFastLap(2.0);
    }
    while (follow.getDisplacement()<2.00){
        follow.updateFastLap(0.80);
    }
    while (follow.getDisplacement()<2.20){
        follow.updateFastLap(2.00);
    }
    while (follow.getDisplacement()<2.65){
        follow.updateFastLap(0.80);  
    }
    while (follow.getDisplacement()<4.70){
        follow.updateFastLap(2.00);
    }
    while (follow.getDisplacement()<5.80){
        follow.updateFastLap(0.80);
    }
    while (follow.getDisplacement()<6.00){
        follow.updateFastLap(1.80);
    }
    while (follow.getDisplacement()<6.70){
        follow.updateFastLap(0.80);
    }
    while (follow.getDisplacement()<6.90){
        follow.updateFastLap(1.80);
    }
    while (follow.getDisplacement()<7.50){
        follow.updateFastLap(0.80);
    }
    while (follow.getDisplacement()<9.40){
        follow.updateFastLap(1.50);
    }
    while (follow.getDisplacement()<10.10){
        follow.updateFastLap(0.90);
    }
    while (follow.getDisplacement()<16.5){
        follow.updateFastLap(0.0);
    }
    follow.stop();
}
