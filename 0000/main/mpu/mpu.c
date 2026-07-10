#include "mpu.h"
#include <stdbool.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "portmacro.h"

QueueHandle_t mpu_queue;

uint8_t data_gyro[6];
uint8_t data_temp[2];
uint8_t data_acc[6];
volatile bool calibration = false;
uint16_t iter = 0;

float sample_period = 0.001f;
const float z_acc_corrector = 14418.0;

void setup_mpu(){
    // Reg 6B is power management 1 -> Clock Source is Internal 8MHz oscillator, temperatura sensor is able, sleep mode is disable.
    mpuWriteReg(MPU6050_REGISTER_PWR_MGMT_1, 0x00, MPU_ADDR);
    mpuWriteReg(MPU6050_REGISTER_INT_ENABLE, 0x00, MPU_ADDR); // Interruption enable, this line dont enable any interruption, just for knowledge
    mpuWriteReg(MPU6050_REGISTER_USER_CTRL, 0x00, MPU_ADDR); // User control,
    mpuWriteReg(MPU6050_REGISTER_FIFO_EN, 0x00, MPU_ADDR); // Fifo enable,
    mpuWriteReg(MPU6050_REGISTER_INT_PIN_CFG, 0x00, MPU_ADDR); // Interruption pin/byspass Enable Config
    mpuWriteReg(MPU6050_REGISTER_SMPLRT_DIV, 0x07, MPU_ADDR); // Reg 19 is Sampe Rate Divider -> with data = 7 -> Sample Rate will be 1kHz; all measurements is always updated at the sample rate configurated
    mpuWriteReg(MPU6050_REGISTER_GYRO_CONFIG, 0x00, MPU_ADDR);
    mpuWriteReg(MPU6050_REGISTER_ACCEL_CONFIG, 0x00, MPU_ADDR);
}

void kalman_filter(kalman_filter_t* k, float gyro_rate, float acc_angle,
																	  float Ts){
    if (Ts <= 0.0f || Ts > 1.0f) {
        return;
    }
    float angle = k->x[0];
    float bias  = k->x[1];

//    Prediction:
    float rate = gyro_rate - bias;
    float angle_pred = angle + Ts * rate;
    float bias_pred  = bias;

//    Covariance prediction
    k->P[0][0] = k->P[0][0] - Ts * k->P[1][0] - Ts * k->P[0][1] + (Ts * Ts *
															k->P[1][1]) + k->Rw;
    k->P[0][1] = k->P[0][1] - Ts * k->P[1][1];
    k->P[1][0] = k->P[1][0] - Ts * k->P[1][1];
    k->P[1][1] = k->P[1][1] + k->Rw_bias;

//    Measurement update
    float y = acc_angle - angle_pred;
    float S = k->P[0][0] + k->Rv;

    k->K[0] = k->P[0][0] / S;
    k->K[1] = k->P[1][0] / S;
    k->x[0] = angle_pred + k->K[0] * y;
    k->x[1] = bias_pred  + k->K[1] * y;

//    Covariance update
    k->P[0][0] -= k->K[0] * k->P[0][0];
    k->P[0][1] -= k->K[0] * k->P[0][1];
    k->P[1][0] -= k->K[1] * k->P[0][0];
    k->P[1][1] -= k->K[1] * k->P[0][1];
}

bool get_orientation(mpu6050_t* mpu, k_filter_orientation_t* k, float Ts){
    float xg_gyro, yg_gyro, zg_gyro;
    float xg_acc, yg_acc, zg_acc;

    mpuReadfromReg(MPU6050_REGISTER_GYRO_XOUT_H, data_gyro, 6, MPU_ADDR);
    mpuReadfromReg(MPU6050_REGISTER_TEMP_OUT_H, data_temp, 2, MPU_ADDR);
    mpuReadfromReg(MPU6050_REGISTER_ACCEL_XOUT_H, data_acc, 6, MPU_ADDR);

    //Get raw measurements of gyro
    int16_t RawX_gyro = (data_gyro[0]<<8)|data_gyro[1];
    int16_t RawY_gyro = (data_gyro[2]<<8)|data_gyro[3];
    int16_t RawZ_gyro = (data_gyro[4]<<8)|data_gyro[5];
    //divide the raw measurement by scale of gyroscope
    xg_gyro = (float)RawX_gyro/131.0;
    yg_gyro = (float)RawY_gyro/131.0;
    zg_gyro = (float)RawZ_gyro/131.0;
    //Get raw measurements of accelerometer
    int16_t RawX_acc = (data_acc[0]<<8)|data_acc[1];
    int16_t RawY_acc = (data_acc[2]<<8)|data_acc[3];
    int16_t RawZ_acc = (data_acc[4]<<8)|data_acc[5];
    //divide the raw measurement by scale of accelerometer
    xg_acc = (float)RawX_acc/16384.0;
    yg_acc = (float)RawY_acc/16384.0;
    zg_acc = (float)RawZ_acc/z_acc_corrector; // idk why xD

    //temperature by mpu6050
    int16_t temp_raw = (data_temp[0]<<8)|data_temp[1];
    mpu->temp = (((float)(temp_raw)/340)+36.53);

    //pitch and roll angles by accelerometer
    mpu->pitch = atan2f(xg_acc, sqrtf((yg_acc * yg_acc) + (zg_acc * zg_acc))) *
																	 180.0/M_PI;
    mpu->roll = atan2f(yg_acc, zg_acc)*180.0/M_PI;

    //kalman filter apply
    kalman_filter(k->pitch, yg_gyro, mpu->pitch, Ts);
    kalman_filter(k->roll, xg_gyro, mpu->roll, Ts);

    mpu->pitch = k->pitch->x[0];
    mpu->roll = k->roll->x[0];

    if(!calibration){
        mpu->offset_roll  += mpu->roll;
        mpu->offset_pitch += mpu->pitch;
        iter++;
        if(iter >= CALIBRATION_SAMPLES){
            mpu->offset_pitch /= (float)CALIBRATION_SAMPLES;
            mpu->offset_roll  /= (float)CALIBRATION_SAMPLES;
            calibration = true;
            iter = 0;
        }
    }else{
        mpu->pitch -= mpu->offset_pitch;
        mpu->roll  -= mpu->offset_roll;
    }

    return calibration;
}

void mpu_task(void* pvParameters){
    mpu6050_t* mpu = (mpu6050_t*)pvParameters;

	TickType_t last_wake_time = xTaskGetTickCount();
	TickType_t period_ticks = pdMS_TO_TICKS(2);

	//kalman filter init
	k_filter_orientation_t K_orientation;

	kalman_filter_t Kfilter_pitch = {
		.x = {0.0f, 0.0f},
		.P = {
			{1.0f, 0.0f},
		    {0.0f, 1.0f}
		},
		.Rv = 0.5f,
		.Rw = 0.001f,
		.Rw_bias = 0.003f
	};

	kalman_filter_t Kfilter_roll = {
		.x = {0.0f, 0.0f},
		.P = {
			{1.0f, 0.0f},
			{0.0f, 1.0f}
		},
		.Rv = 0.5f,
		.Rw = 0.001f,
		.Rw_bias = 0.003f
	};

    K_orientation.pitch = &Kfilter_pitch;
    K_orientation.roll  = &Kfilter_roll;

    unsigned long last_time = esp_timer_get_time();

	if(!calibration)
			ESP_LOGW(TAG_MPU, "Calibrating...");

    while(1){
        unsigned long current_time = esp_timer_get_time();
        float Ts = (current_time - last_time)/1e6; // in seconds
        last_time = current_time;

		if (Ts <= 0.0f || Ts > 0.1f) {
		    Ts = 0.002f; // 2ms
		}

        calibration = get_orientation(mpu, &K_orientation, Ts);

        if(calibration){
			xQueueSend(mpu_queue, mpu, 0);
            //ESP_LOGI(TAG_MPU, "PITCH: %.2f ROLL: %.2f\n", mpu->pitch, mpu->roll);
        }
		//ensure 2 ms for the mpu task, 500 Hz
		//vTaskDelayUntil(&last_wake_time, period_ticks);
    }
}
