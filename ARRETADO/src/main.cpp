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

int count = 0;
int contb = 0; 

float velocity[100] =  {0.8,2,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,2,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,2,0.8,0.8,0.8,0.8,0.8,0.8,2,0.8,0.8,0.8,0.8,2,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,2,0.8,0.8,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
 
float distance[100] = {0,1.0355,1.3807,1.5411,1.7121,1.8041,1.918,2.2335,2.3387,2.5796,4.3526,4.9436,5.0524,5.4238,5.5151,5.8453,6.0606,6.1324,6.4071,6.8363,6.9794,7.1507,7.4895,7.6491,7.7024,8.6767,9.1624,9.3243,9.5846,10.3228,10.6029,11.0362,11.1806,11.3605,11.6926,12.2707,12.3585,12.5586,13.315,13.3546,13.8958,13.9856,14.3228,14.7536,15.2015,15.3508,15.418,16.0003,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


int main()
{
    wait(1);

    contb = follow.ButtonPress();

    if(contb==1){
        follow.start();
        while(follow.getDisplacement()<16.5){
            follow.updateMapLap(0.6);
        }
        follow.stop();
        follow.Map();
        follow.bluetooth();
    }

    if(contb==2){
        follow.start();
        while(follow.getDisplacement()<16.5){
             while (follow.getDisplacement()<distance[count])
            {
                follow.updateFastLap(velocity[count]);
            }
            count++;
        }
        follow.stop();
    }
}
