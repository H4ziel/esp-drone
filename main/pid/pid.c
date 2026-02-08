#include "pid.h"
#include <stdlib.h>
#include <math.h>

void pid_init(pid_t* p){
    if(p == NULL) return;

    p->kp = 15.0;
    p->ki = 0.05;
    p->kd = 2.0;
    p->i_error = 0;
    p->MAX = 1500.0f; //
    p->MIN = -1500.0f;
    p->last_error = 0;
}

float pid_control(pid_t* p, float set_point, float current_meas, float Ts){
    float error, P, I, D, out;

    error = set_point - current_meas;

    P = error * p->kp;

    p->i_error += error*Ts;
	I = p->ki * p->i_error;

    if(I > (p->MAX/5)){
        I = p->MAX/5;
        p->i_error = (p->MAX/5) / p->ki;
    }
    if(I < (p->MIN/5)){
        I = p->MIN/5; //~20% of min out
        p->i_error = (p->MIN/5) / p->ki;
    }
    D = p->kd * ((error - p->last_error) / Ts);

    p->last_error = error;

    out = P + I + D;

    if(p->MAX < out) out = p->MAX;
    if(p->MIN > out) out = p->MIN;

    return out;
}
