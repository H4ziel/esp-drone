#ifndef _MPU_H_INCLUDED
#define _MPU_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "esp_err.h"
#include "mpu6050_regs.h"
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "i2c/i2c_dr.h"

#define MPU_ADDR 			0x68
#define CALIBRATION_SAMPLES 5000

static const char* TAG_MPU  = "MPU6050";

typedef struct {
    float Rv;      // noise by measurements, accelerometer
    float Rw;      // noise by angle/model
    float Rw_bias; // noise by gyro bias
    float x[2];    // x[0] = angle, x[1] = gyro bias
    float P[2][2]; // covariance matrix
    float K[2];    // Kalman gain
} kalman_filter_t;

typedef struct{
    kalman_filter_t* roll;
    kalman_filter_t* pitch;
}k_filter_orientation_t;

typedef struct{
    float pitch;
    float roll;
    float yaw;
}orientation_t;

typedef struct MPU6050{
    orientation_t orientation;
    orientation_t offset_orientation;
    orientation_t setpoint_orientation;
    float temp;
}mpu6050_t;

void kalman_filter(kalman_filter_t* k, float gyro, float acc_angle, float Ts);

void setup_mpu(void);
bool get_orientation(mpu6050_t* mpu, k_filter_orientation_t* k, float Ts);
void mpu_task(void* pvParameters);

#endif
