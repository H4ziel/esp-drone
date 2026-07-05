#ifndef _MPU_H_INCLUDED
#define _MPU_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"
#include "hal/i2c_types.h"
#include "mpu6050_regs.h"
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22
#define I2C_NUM I2C_NUM_0

typedef unsigned char byte;

#define MPU_ADDR 			0x68
#define CALIBRATION_SAMPLES 500

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

typedef struct MPU6050{
    float roll;
    float pitch;
    float yaw;
    float temp;
    float offset_pitch;
    float offset_roll;
}mpu6050_t;

esp_err_t mpuReadfromReg(byte Reg, byte* ReadBuffer, size_t len);
esp_err_t mpuWriteReg(byte Reg, byte data);
esp_err_t i2c_master_init(void);
void setup_i2c(void);

void kalman_filter(kalman_filter_t* k, float gyro, float acc_angle, float Ts);

void setup_mpu(void);
bool get_orientation(mpu6050_t* mpu, k_filter_orientation_t* k, float Ts);
void mpu_task(void* pvParameters);

#endif
