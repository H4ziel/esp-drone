#ifndef PID_H
#define PID_H

#include "freertos/idf_additions.h"
#include "pwm.h"
#include <stdlib.h>
#include <math.h>
#include "mpu.h"

typedef struct PID{
    float kp;
    float ki;
    float kd;
    float i_error, MAX, MIN, last_error;
}pid_t;

const static char* TAG_PID = "PID";

float clamp_float(float value, float min, float max);
void pid_init(pid_t* p, float KP, float KI, float KD);
float pid_control(pid_t* p, float set_point, float current_meas, float Ts);
void pid_task(void *pvParameters);

#endif
