#include "Follow.h"

//Motor PWM
TB6612 leftPWM(PWMA, AIN1 ,AIN2); 
TB6612 rightPWM(PWMB, BIN1, BIN2);           

//Encoders
QEI leftEncoder(OUTA1, OUTB1, NC, 120, QEI::X4_ENCODING);   //chanA, chanB, index, ppr 
QEI rightEncoder(OUTA2, OUTB2, NC, 120, QEI::X4_ENCODING);  //chanA, chanB, index, ppr ->12 readings per revolution * Gear Ratio=1:10 

//Motor Controlled Speed
MotorControl leftMotor(1.7, 0.000005, 0, &leftPWM, &leftEncoder);     //kP, kI, kD, looptime, leftPWM  address, leftEncoder  address 
MotorControl rightMotor(2, 0.000005, 0, &rightPWM, &rightEncoder);//kP, kI, kD, RightPWM address, RightEncoder address

//Follow Class----angular velocity, sensor bar, curve radius...
Follow follow(0.1, 0, 0.001, &leftMotor, &rightMotor, 20); // lt = 30 kp = 0.1055 --- P. madrugada 0.065, 0.00026, 2 ------


float Mark[30]     = 
{
0.9,2.55,2.95,3.9,4.167,5.517,5.784,7.264,7.264,7.531,7.971,8.521,9.055,9.445,9.845,10.265,10.665,12.315,13.115,13.905,14.705,15.495,16.295,17.085,17.885,18.675,19.075,19.865,20.765
,30
};       
float setSpeed[30] = 
{ 
0.6, 2, 1, 1.8, 1, 1.8, 1, 1.8, 1.4, 1.8, 1.8,1.4,1.4,1.4,1.4,1.4,1.4,1.4,1.4,1.4,1.4,1.4,1.4,1.4,1.4,1.4,1.4,1.4,1.4
,0

};   //setpoint for linear speed [m/s]
//float Radius[30];     //curve radius [m]
bool a = 0;
int main() {
    
    wait(1);
    millisStart();
    
    int count = 0;
    while(1){
        
        follow.update(1);
        
        
    //    if(((millis()-tempoleitura)>100) and follow.getMark()){
    //        count ++;
    //        tempoleitura = millis();
        
        if(Mark[count]<=follow.getDisplacement()){
            count ++;
            a = !a;
            follow.Buzzer(a);
        }
    }

}
    
