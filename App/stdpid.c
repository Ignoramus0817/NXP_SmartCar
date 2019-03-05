
/*
 * name: stdpid.c
 * title: "standard" PID controller
 * author: Fullstack-MHY 16061023
 * recommended IDE: IAR Embedded Workbench 8.0
 */

#include "stdpid.h"
float stdPID(struct PID_Para _Para, float _input)
{
    float output;
    // PID terms
    static float Calc_P, Calc_I, Calc_D;
    // error buffer
    static float err[2] = {0, 0};
    
    // refresh error buffer
    err[1] = err[0];
    err[0] = _input;
    
    // calculate PID terms
    Calc_P = err[0];
    Calc_I += _Para.time*err[0];
    Calc_D = (err[0] - err[1])/_Para.time;

    // intergration limitation
    if(Calc_I > _Para.I_limit)
        Calc_I = _Para.I_limit;
    else if(Calc_I < -_Para.I_limit)
        Calc_I = -_Para.I_limit;
    
    // calculate PID output
    output = _Para.P*Calc_P + _Para.I*Calc_I + _Para.D*Calc_D;

    // output limitation
    if(output > _Para.Output_limit)
        output = _Para.Output_limit;
    else if(output < -_Para.Output_limit)
        output = -_Para.Output_limit;

    return output;
}