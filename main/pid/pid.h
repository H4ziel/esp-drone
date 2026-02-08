#ifndef PID_H
#define PID_H

typedef struct PID{
    float kp;
    float ki;
    float kd;
    float i_error, MAX, MIN, last_error;
}pid_t;

void pid_init(pid_t* p);
float pid_control(pid_t* p, float set_point, float current_meas, float Ts);

#endif
