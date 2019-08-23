/*#include "Follow.h"
#include "mbed.h"


//Motor PWM
*/TB6612 leftPWM(PWMA, AIN1, AIN2);
TB6612 rightPWM(PWMB, BIN1, BIN2);

//Encoders
QEI leftEncoder(OUTA1, OUTB1, NC, 120, QEI::X4_ENCODING);  //chanA, chanB, index, ppr
QEI rightEncoder(OUTA2, OUTB2, NC, 120, QEI::X4_ENCODING); //chanA, chanB, index, ppr ->12 readings per revolution * Gear Ratio=1:10

//Motor Controlled Speed
MotorControl leftMotor(1.75, 0.000005, 0, &leftPWM, &leftEncoder);  //kP, kI, kD, looptime, leftPWM  address, leftEncoder  address
//                     3.2, 0.000005, 0 --> bat 8.00
MotorControl rightMotor(1.9, 0.000005, 0, &rightPWM, &rightEncoder); //kP, kI, kD, RightPWM address, RightEncoder address

//Follow Class----angular velocity, sensor bar, curve radius...
Follow follow(0.045, 0, 0.06 , &leftMotor, &rightMotor, 20);//kp, ki, kd, leftmotor(object), rightmotor(object), looptime
//            0.025, 0, 0.06 --> bat 8.00

int count = 0;

int contb = 0; 

//float velocity[100] = {0.9,2.0,0.8,0.8,1.5,0.8,0.8,1.8,0.8,0.8,2.0,0.8,0.8,1.5,0.8,1.5,2.0,0.8,0.8,2.0,0.8,0.8,1.5,1.5,0.8,0.8,2.0,0.8,0.8,2.0,0.8,1.5,0.8,0.8,1.5,2.0,0.8,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,2.0,0.8,0.8,2.0,0.8,0.8,0.8,0.8,2.0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//float distance[100] = {0,1.0634,1.418,1.5808,1.7576,1.8552,1.9744,2.2981,2.406,2.6552,4.4702,5.0752,5.2072,5.5896,5.7176,6.0524,6.2759,6.3504,6.642,7.0911,7.2408,7.4288,7.7732,7.9403,7.996,8.9912,9.4958,9.6639,9.9444,10.702,11.0079,11.4607,11.6116,11.8108,12.1468,12.7523,12.8344,13.0515,13.4336,13.4708,13.7414,13.8316,13.8708,13.9989,14.0416,14.0807,14.334,14.4184,14.5256,14.8687,15.3292,15.7927,15.9472,16.018,16.5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

float distance2[100] = {0,1.0355,1.3807,1.5411,1.7121,1.8041,1.918,2.2335,2.3387,2.63,4.3526,4.9436,5.0524,5.4238,5.5151,5.8453,6.0606,6.1324,6.4071,6.75,6.9794,7.1507,7.4895,7.6491,7.7024,8.6767,9.1624,9.25,9.5846,10.2,10.3,11.0362,11.15,11.3605,12.12,12.2707,12.3585,12.45,13.315,13.3546,13.8958,13.9856,14.3228,14.6,15.2015,15.3,15.45,16.0003,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float velocity2[100] =  {2,1.6,0.8,0.8,0.8,0.8,0.8,1.5,0.8,0.8,2.2,0.9,0.8,0.8,0.8,1.8,0.8,0.8,0.8,2,0.5,0.8,1.8,1.4,1.4,1.4,1.4,0.8,0.8,1.5,0.8,1,0.8,0.8,2,0.6,0.8,0.8,0.7,0.7,0.7,1.5,1.5,0.8,1.2,1.2,1,1.5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

float velocity[100] =  {2,2,0.8,0.8,0.8,0.8,0.8,1.5,0.8,0.8,2.2,0.9,0.8,0.8,0.8,1.8,0.8,0.8,0.8,2,0.5,0.8,1.8,1.4,1.4,1.4,1.4,0.8,0.8,1.5,0.8,1,0.8,0.8,1.8,0.8,0.8,0.8,0.7,0.7,0.7,1.5,1.5,0.8,1.2,1.2,1,1.5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float distance[100] = {0,1.0355,1.3807,1.5411,1.7121,1.8041,1.918,2.2335,2.3387,2.63,4.3526,4.9436,5.0524,5.4238,5.5151,5.8453,6.0606,6.1324,6.4071,6.75,6.9794,7.1507,7.4895,7.6491,7.7024,8.6767,9.1624,9.25,9.5846,10.2,10.3,11.0362,11.15,11.45,12.2,12.2707,12.3585,12.45,13.315,13.3546,13.8958,13.9856,14.3228,14.6,15.2015,15.3,15.45,16.3003,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

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
        while(follow.getDisplacement()<16.8){
             while (follow.getDisplacement()<distance[count])
            {
                follow.updateFastLap(velocity[count]);
            }
            count++;
        }
        follow.stop();
    }

    if(contb==3){
        follow.start();
        while(follow.getDisplacement()<16.5){
             while (follow.getDisplacement()<distance2[count])
            {
                follow.updateFastLap(velocity2[count]);
            }
            count++;
        }
        follow.stop();
    }

    if(contb==4){
        follow.start();
        while(follow.getDisplacement()<12.44){
             while (follow.getDisplacement()<distance[count])
            {
                follow.updateFastLap(velocity[count]);
            }
            count++;
        }

        while(1){
            follow.stop();
        }

        while(follow.getDisplacement()<13.9){
            follow.straightFoward(1);
        }

        while(follow.getDisplacement()<16.5){
            follow.updateFastLap(0.8);
            count++;
        }
    }
    
}*/

#include <mbed.h>

AnalogIn Sensor1(A0);

AnalogIn Sensor2(A1);

AnalogIn Sensor3(A2);

AnalogIn Sensor4(A3);

AnalogIn Sensor5(A4);

AnalogIn Sensor6(A5);

AnalogIn Sensor7(A6);

//AnalogIn Sensor8(A7);

 

int main(){

  while(1){

    int sen1 = (Sensor1>0.5);

    int sen2 = (Sensor2>0.5);

    int sen3 = (Sensor3>0.5);

    int sen4 = (Sensor4>0.5);

    int sen5 = (Sensor5>0.8);

    int sen6 = (Sensor6>0.8);

    int sen7 = (Sensor7>0.9);

    //int sen8 = (Sensor8>0.5);
 

    printf("%i ", sen1 );

    printf("%i ", sen2 );

    printf("%i ", sen3 );

    printf("%i ", sen4 );

    printf("%i ", sen5 );

    printf("%i ", sen6 );

    printf("%i ", sen7 );

    //printf("%i ", sen8 );

    printf("\n");

  }

}