/* wallbot mini Library
 *
 * wallbotmini.cpp
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


#include "TB6612.h"

// TB6612 Class Constructor
TB6612::TB6612(PinName pwm, PinName fwd, PinName rev):
        _pwm(pwm), _fwd(fwd), _rev(rev) {

    _fwd = 0;
    _rev = 0;
    _pwm = 0.0;
    stat = 0;
    _pwm.period(0.001);
    
}

// Speed Control
//  arg
//   float speed -1.0 - 0.0 - 1.0
float TB6612::speed(float speed) {
    
    if( speed > 0.0 )
    {
        _pwm = speed;
        _fwd = 1;
        _rev = 0;
        stat = 1;
    }
    else if( speed < 0.0 )
    {
        _pwm = -speed;
        _fwd = 0;
        _rev = 1;
        stat = -1;
    }
    else
    {
        _fwd = 0;
        _rev = 0;
        stat = 1;
    }
    return speed==0 ? 0 : speed > 0 ? 1 : -1;
}