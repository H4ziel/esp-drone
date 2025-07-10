#include "pid.h"
#include <stdlib.h>

pid_t* pid_init(pid_t* p){
    p = (pid_t*)malloc(sizeof(pid_t));
    p->kp = 0.9;
    p->ki = 0.06;
    p->kd = 0.0001;
    p->i_error = 0;
    p->MAX = 500;
    p->MIN = -500;
    p->last_error = 0;

    return p;
}

float pid_control(pid_t* p, float set_point, float current_meas, float Ts){
    float error, P, I, D, out;

    error = set_point - current_meas;

    P = error * p->kp;

    p->i_error += error*Ts;

    I = p->ki * p->i_error;

    D = p->kd * ((error - p->last_error) / Ts);

    p->last_error = error;

    out = P + I + D;

    if(p->MAX < out){
        p->i_error = 0;
        out = p->MAX;
    }else if(p->MIN > out){
        p->i_error = 0;
        out = p->MIN;
    }

    return out;
}
