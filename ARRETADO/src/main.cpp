#include "Follow.h"
#include "mbed.h"


//Motor PWM
TB6612 leftPWM(PWMA, AIN1, AIN2);
TB6612 rightPWM(PWMB, BIN1, BIN2);

//Encoders
QEI leftEncoder(OUTA1, OUTB1, NC, 120, QEI::X4_ENCODING);  //chanA, chanB, index, ppr
QEI rightEncoder(OUTA2, OUTB2, NC, 120, QEI::X4_ENCODING); //chanA, chanB, index, ppr ->12 readings per revolution * Gear Ratio=1:10

//Motor Controlled Speed
MotorControl leftMotor(2.0, 0.000005, 0, &leftPWM, &leftEncoder);  //kP, kI, kD, looptime, leftPWM  address, leftEncoder  address
//                     3.2, 0.000005, 0 --> bat 8.00
MotorControl rightMotor(2.0, 0.000005, 0, &rightPWM, &rightEncoder); //kP, kI, kD, RightPWM address, RightEncoder address

//Follow Class----angular velocity, sensor bar, curve radius...
Follow follow(0.028, 0, 0.06 , &leftMotor, &rightMotor, 20);//kp, ki, kd, leftmotor(object), rightmotor(object), looptime
//            0.025, 0, 0.06 --> bat 8.00


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

    while (follow.getDisplacement()<1.4){
        follow.updateFastLap(0.3);
    }
    follow.stop();
    while(1){
        wait(1);
    }
    while (follow.getDisplacement()<0.4){
        follow.updateFastLap(1.2);
    }
    while (follow.getDisplacement()<1.0){
        follow.updateFastLap(1.6);
    }
    while (follow.getDisplacement()<1.2){
        follow.updateFastLap(1.0);  
    }
    while (follow.getDisplacement()<1.3){
        follow.updateFastLap(0.5);
    }
    while (follow.getDisplacement()<2.77){
        follow.updateFastLap(0.5);
    }
    while (follow.getDisplacement()<2.87){
        follow.updateFastLap(1.0);
    }
    while (follow.getDisplacement()<2.97){
        follow.updateFastLap(1.8);
    }
    while (follow.getDisplacement()<4.84){
        follow.updateFastLap(1.2);
    }
    while (follow.getDisplacement()<4.94){
        follow.updateFastLap(0.6);
    }
    while (follow.getDisplacement()<5.05){
        follow.updateFastLap(0.3);
    }
    while (follow.getDisplacement()<7.30){
        follow.updateFastLap(0.6);
    }
    while (follow.getDisplacement()<16.5){
        follow.updateFastLap(0.6);
    }
    follow.stop();
}
