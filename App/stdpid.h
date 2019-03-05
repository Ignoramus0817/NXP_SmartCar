#ifndef __STDPID__
#define __STDPID__
struct PID_Para
{
    float P, I, D;
    float time;
    float I_limit;
    float Output_limit;
};
float stdPID(struct PID_Para _Para, float _input);
#endif