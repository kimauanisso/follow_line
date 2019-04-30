/* mbed TB6612FNG Library
 *
 * TB6612.h
 *
 * Copyright (c) 2010-2013 jksoft
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef MBED_TB6612_H
#define MBED_TB6612_H

#include "mbed.h"

/** TB6612FNG  Library
 *
 * Example:
 * @code
 * // Drive the Motor

#include "mbed.h"
#include "TB6612.h"

TB6612 motor(p21,p5,p6);    // PWMA,AIN1,AIN2
 
int main() {

    motor = 0.0;        // Motor stopped.

    while(1)
    {
        motor = 0.5;    // Motor forward.
        wait(2.0);
        motor = -0.5;   // Motor reversal.
        wait(2.0);
    }
 }

 * @endcode
 */

class TB6612 {
    // Public functions
public:
    /** Create a TB6612 connected to the specified pins.
     * @param pwm A PwmOut pin, driving the H-bridge enable line to control the speed.（PwmOutに対応したポートを指定します。）
     * @param fwd A DigitalOut, set high when the motor should go forward.（DigitalOutに対応したポートを指定します。）
     * @param rev A DigitalOut, set high when the motor should go backwards.（DigitalOutに対応したポートを指定します。）
     */
    TB6612(PinName pwm, PinName fwd, PinName rev);
    /** Directly control the speed and direction of the motor
     *
     * @param speed A normalised number -1.0 - 1.0 represents the full range.
     * @return return the stopped state or direction of rotation.
     */
    float speed(float speed);
    /** A operator shorthand for speed()
     *
     */ 
    void operator= ( float value )
    {
        speed(value);
    }
    operator int()
    {
        return(stat);
    }
    
protected:
    PwmOut _pwm;
    DigitalOut _fwd;
    DigitalOut _rev;
    int stat;
};

#endif
