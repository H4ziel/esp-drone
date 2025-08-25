#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "driver/adc_types_legacy.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "hal/adc_types.h"
#include "hal/i2c_types.h"
#include "portmacro.h"
#include "pwm/pwm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "mpu6050_regs.h"
#include "esp_timer.h"
#include "pid/pid.h"
#include "lora.h"

#define _USE_MATH_DEFINES

#define PWM1_PIN GPIO_NUM_2
#define PWM1_TIMER LEDC_TIMER_1
#define PWM1_CHANNEL LEDC_CHANNEL_0

#define PWM2_PIN GPIO_NUM_4
#define PWM2_TIMER LEDC_TIMER_0
#define PWM2_CHANNEL LEDC_CHANNEL_1

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22
#define I2C_NUM I2C_NUM_0

uint8_t MPU_ADDR = 0x68; // or 0x69, AD0 is the least bit of this reg, AD0 -> 3V3 = 1 -> addr = 0x69; AD -> 0V = 0 -> addr = 0x68

static uint16_t dutyc = 0;
float sample_period = 0.001f;
float x_gyro_bias = -2.41;
float y_gyro_bias = -0.39;
float z_gyro_bias = -0.57;
float x_acc_bias = -0.09;
float y_acc_bias = -0.03;
float z_acc_bias = 0.78;
static float gyro_scale = 250;
static uint8_t acc_scale = 2;
const float z_acc_corrector = 14418.0;

static const char* TAG_MPU  = "MPU6050";
static const char* TAG_PWM  = "PWM";
static const char* TAG_PID  = "PID";
static const char* TAG_LORA = "LORA";

typedef struct MPU6050{
    float xg_gyro;
    float yg_gyro;
    float zg_gyro;
    float xg_acc;
    float yg_acc;
    float zg_acc;
    float temp;
    float roll;
    float pitch;
    float yaw;
    float kalman_x;
    float kalman_y;
}mpu6050_t;

typedef struct KalmanFilter{
    float Rv; // noise by measurements (accelerometer)
    float Rw; // noise by model (gyroscope)
    float x[2]; // predicted state: [angle, speed angular]
    float P[2][2]; // covariance matrix of predicted errors -> calculate how much uncertain is the filter
    float M[2][2]; // covariace matrix of propagation of uncertain
    float Gamma[2]; // this matrix indicate how the input influence the state
    float Phi[2][2]; // this matrix indicate the temporal advancement of the state
    float C[2]; // Observation Matrix -> extracts which measurement is needed for the calculation, for this application, we need only the angle of state -> C = {1, 0};
    float I[2][2]; // Idendity matrix
    float K[2]; // Kalman Gain
}kalman_filter_t;

esp_err_t i2c_master_init(void);
mpu6050_t* setup_mpu();
void setup_i2c();
void setup_sx1276();
void kalman_filter(kalman_filter_t* k, float gyro, float acc_angle, float Ts);
esp_err_t mpuReadfromReg(uint8_t Reg, uint8_t* ReadBuffer, size_t len);
esp_err_t mpuWriteReg(uint8_t Reg, uint8_t data);

esp_err_t mpuReadfromReg(uint8_t Reg, uint8_t* ReadBuffer, size_t len){
    return (i2c_master_write_read_device(I2C_NUM, MPU_ADDR, &Reg, 1, ReadBuffer, len, 2000));
}

esp_err_t mpuWriteReg(uint8_t Reg, uint8_t data){
    uint8_t writeBuf[2];
    writeBuf[0] = Reg;
    writeBuf[1] = data;
    return (i2c_master_write_to_device(I2C_NUM, MPU_ADDR, writeBuf, 2, 1000));
}

esp_err_t i2c_master_init(void){
    i2c_config_t i2c_config = {
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .mode = I2C_MODE_MASTER,
        .master.clk_speed = 400000,
    };

    i2c_param_config(I2C_NUM, &i2c_config);
    return (i2c_driver_install(I2C_NUM, i2c_config.mode, 0, 0, 0));
}

void setup_i2c(){
    ESP_ERROR_CHECK(i2c_master_init());
    uint8_t buf;
    mpuReadfromReg(MPU6050_REGISTER_WHO_AM_I, &buf, 1);

    ESP_LOGI(TAG_MPU, "Device addr:0x%x", buf);
}

mpu6050_t* setup_mpu(){
    mpu6050_t* mpu = (mpu6050_t*)malloc(sizeof(mpu6050_t));
    mpu = 0;
    // Reg 6B is power management 1 -> Clock Source is Internal 8MHz oscillator, temperatura sensor is able, sleep mode is disable.
    mpuWriteReg(MPU6050_REGISTER_PWR_MGMT_1, 0x00);
    mpuWriteReg(MPU6050_REGISTER_INT_ENABLE, 0x00); // Interruption enable, this line dont enable any interruption, just for knowledge
    mpuWriteReg(MPU6050_REGISTER_USER_CTRL, 0x00); // User control,
    mpuWriteReg(MPU6050_REGISTER_FIFO_EN, 0x00); // Fifo enable,
    mpuWriteReg(MPU6050_REGISTER_INT_PIN_CFG, 0x00); // Interruption pin/byspass Enable Config
    mpuWriteReg(MPU6050_REGISTER_SMPLRT_DIV, 0x07); // Reg 19 is Sampe Rate Divider -> with data = 7 -> Sample Rate will be 1kHz; all measurements is always updated at the sample rate configurated
    mpuWriteReg(MPU6050_REGISTER_GYRO_CONFIG, 0x00);
    mpuWriteReg(MPU6050_REGISTER_ACCEL_CONFIG, 0x00);
    vTaskDelay(pdMS_TO_TICKS(500));

    return mpu;
}

void setup_sx1276(){
    if(lora_init() == 0){
        ESP_LOGW(TAG_LORA, "SX1276 init failed!");
        return;
    }

    lora_set_frequency(915e6);
    lora_enable_crc();
    lora_set_coding_rate(CONFIG_CODING_RATE);
    lora_set_bandwidth(CONFIG_BANDWIDTH);
    lora_set_spreading_factor(CONFIG_SF_RATE);
}

void pwm_task(void* pvParameters){
    int raw;
    while(true){
        uint16_t result = adc2_get_raw(ADC2_CHANNEL_3, ADC_WIDTH_BIT_12, &raw);
        uint8_t dc_percent = (raw*100)/4095;
        uint32_t dc = (dc_percent * ((1 << PWM_DUTY_RESOLUTION) - 1)) / 100;

        duty_att(dc, PWM2_TIMER, PWM2_CHANNEL);

        ESP_LOGI(TAG_PWM,"dc2: %ld", ledc_get_duty(LEDC_HIGH_SPEED_MODE,
                                                                 PWM2_CHANNEL));

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    vTaskDelete(NULL);
}

void kalman_filter(kalman_filter_t* k, float gyro, float acc_angle, float Ts){
    k->Phi[0][0] = 1;
    k->Phi[0][1] = Ts;
    k->Phi[1][0] = 0;
    k->Phi[1][1] = 1;
    k->Gamma[0] = Ts/2;
    k->Gamma[1] = 1;

    //prediction step
    float x_pred[2];
    x_pred[0] = k->Phi[0][0]*k->x[0] + k->Phi[0][1]*k->x[1] + k->Gamma[0]*gyro;
    x_pred[1] = k->Phi[1][0]*k->x[0] + k->Phi[1][1]*k->x[1] + k->Gamma[1]*gyro;

    //prediction of covariance step
    k->M[0][0] = k->Phi[0][0]*k->P[0][0] + k->Phi[0][1]*k->P[1][0];
    k->M[0][1] = k->Phi[0][0]*k->P[0][1] + k->Phi[0][1]*k->P[1][1];
    k->M[1][0] = k->Phi[1][0]*k->P[0][0] + k->Phi[1][1]*k->P[1][0];
    k->M[1][1] = k->Phi[1][0]*k->P[0][1] + k->Phi[1][1]*k->P[1][1];

    //add Noise of model
    k->M[0][0] += k->Rw;
    k->M[1][1] += k->Rw;

    //kalman gain calculate
    float S = k->M[0][0] + k->Rv; // innovation variation
    k->K[0] = k->M[0][0] / S;
    k->K[1] = k->M[1][0] / S;

    //state correction
    float y = acc_angle - x_pred[0];
    k->x[0] = x_pred[0] + k->K[0]*y;
    k->x[1] = x_pred[1] + k->K[1]*y;

    //covariance correction
    k->P[0][0] = (1 - k->K[0]) * k->M[0][0];
    k->P[0][1] = (1 - k->K[0]) * k->M[0][1];
    k->P[1][0] = k->M[1][0] - k->K[1] *k->M[0][0];
    k->P[1][1] = k->M[1][1] - k->K[1] * k->M[0][1];
}

void mpu_task(void* pvParameters){
    mpu6050_t* mpu = (mpu6050_t*)pvParameters;
    uint8_t data_gyro[10];
    uint8_t data_temp[2];
    uint8_t data_acc[10];

    //kalman filter init
    kalman_filter_t* Kfilter_pitch = (kalman_filter_t*)malloc(sizeof(kalman_filter_t));
    kalman_filter_t* Kfilter_roll = (kalman_filter_t*)malloc(sizeof(kalman_filter_t));
    Kfilter_pitch->x[0] = 0;
    Kfilter_pitch->x[1] = 0;
    Kfilter_pitch->I[0][0] = 1;
    Kfilter_pitch->I[0][1] = 0;
    Kfilter_pitch->I[1][0] = 0;
    Kfilter_pitch->I[1][1] = 1;
    Kfilter_pitch->C[0] = 1;
    Kfilter_pitch->C[1] = 0;
    Kfilter_pitch->Rv = 0.03;
    Kfilter_pitch->Rw = 0.001;
    Kfilter_roll->x[0] = 0;
    Kfilter_roll->x[1] = 0;
    Kfilter_roll->I[0][0] = 1;
    Kfilter_roll->I[0][1] = 0;
    Kfilter_roll->I[1][0] = 0;
    Kfilter_roll->I[1][1] = 1;
    Kfilter_roll->C[0] = 1;
    Kfilter_roll->C[1] = 0;
    Kfilter_roll->Rv = 0.03;
    Kfilter_roll->Rw = 0.001;

    unsigned long last_time = 0;

    while(1){
        unsigned long current_time = esp_timer_get_time();
        float Ts = (current_time - last_time)/1e6; // in seconds
        last_time = current_time;

        mpuReadfromReg(MPU6050_REGISTER_GYRO_XOUT_H, data_gyro, 6);
        vTaskDelay(pdMS_TO_TICKS(50));
        mpuReadfromReg(MPU6050_REGISTER_TEMP_OUT_H, data_temp, 2);
        vTaskDelay(pdMS_TO_TICKS(50));
        mpuReadfromReg(MPU6050_REGISTER_ACCEL_XOUT_H, data_acc, 6);

        //Get raw measurements of gyro
        int16_t RawX_gyro = (data_gyro[0]<<8)|data_gyro[1];
        int16_t RawY_gyro = (data_gyro[2]<<8)|data_gyro[3];
        int16_t RawZ_gyro = (data_gyro[4]<<8)|data_gyro[5];
        //divide the raw measurement by scale of gyroscope
        mpu->xg_gyro = (float)RawX_gyro/131.0;
        mpu->yg_gyro = (float)RawY_gyro/131.0;
        mpu->zg_gyro = (float)RawZ_gyro/131.0;
        //Get raw measurements of accelerometer
        int16_t RawX_acc = (data_acc[0]<<8)|data_acc[1];
        int16_t RawY_acc = (data_acc[2]<<8)|data_acc[3];
        int16_t RawZ_acc = (data_acc[4]<<8)|data_acc[5];
        //divide the raw measurement by scale of accelerometer
        mpu->xg_acc = (float)RawX_acc/16384.0;
        mpu->yg_acc = (float)RawY_acc/16384.0;
        mpu->zg_acc = (float)RawZ_acc/z_acc_corrector; // idk why xD

        //calibration -> off-set correction
        mpu->xg_gyro = (mpu->xg_gyro - x_gyro_bias)*sample_period;
        mpu->yg_gyro = (mpu->yg_gyro - y_gyro_bias)*sample_period;
        mpu->zg_gyro = (mpu->zg_gyro - z_gyro_bias)*sample_period;
        mpu->xg_acc = (mpu->xg_acc - x_acc_bias);
        mpu->yg_acc = (mpu->yg_acc - y_acc_bias);
        mpu->zg_acc = (mpu->zg_acc - z_acc_bias);

        //temperature by mpu6050
        int16_t temp_raw = (data_temp[0]<<8)|data_temp[1];
        mpu->temp = (float)((temp_raw/340)+36.53);

        //pitch and roll angles by accelerometer
        mpu->pitch = asinf(mpu->xg_acc)*180.0/M_PI;
        mpu->roll = atan2f(mpu->yg_acc, mpu->zg_acc)*180.0/M_PI;

        //kalman filter apply
        kalman_filter(Kfilter_pitch, mpu->yg_gyro, mpu->pitch, Ts);
        kalman_filter(Kfilter_roll, mpu->xg_gyro, mpu->roll, Ts);

        mpu->pitch = Kfilter_pitch->x[0];
        mpu->roll = Kfilter_roll->x[0];

        ESP_LOGI(TAG_MPU, "Temp: %.2f", mpu->temp);
        //ESP_LOGI(TAG_MPU, "x_gyro=%.2f  y_gyro=%.2f  z_gyro=%.2f", mpu->xg_gyro,mpu->yg_gyro, mpu->zg_gyro);
        //ESP_LOGI(TAG_MPU, "x_acc=%.2f y_acc=%.2f z_acc=%.2f", mpu->xg_acc, mpu->yg_acc, mpu->zg_acc);
        //ESP_LOGI(TAG_MPU, "pitch_acc=%.1f roll_acc=%.1f", pitch_angle_acc, roll_angle_acc);
        ESP_LOGI(TAG_MPU, "pitch=%.1f roll=%.1f yaw=%.1f", mpu->pitch, mpu->roll, mpu->yaw);
        vTaskDelay(pdMS_TO_TICKS(0.1));
    }
    free(mpu);
    free(Kfilter_roll);
    free(Kfilter_pitch);
    vTaskDelete(NULL);
}

void pid_task(void* pvParameters){
    mpu6050_t* mpu = (mpu6050_t*)pvParameters;

    pid_t* pitch_pid = pid_init();
    pid_t* roll_pid = pid_init();

    float pitch_pid_out;
    float roll_pid_out;
    unsigned long last_time = 0;

    while(1){
        unsigned long current_time = esp_timer_get_time();
        float Ts = (current_time - last_time)/1e6;
        last_time = current_time;

        pitch_pid_out = pid_control(pitch_pid, 0.0f, mpu->pitch, Ts);
        roll_pid_out = pid_control(roll_pid, 0.0f, mpu->roll, Ts);

        ESP_LOGI(TAG_PID, "Pitch pid:%.2f", pitch_pid_out);
        ESP_LOGI(TAG_PID, "Roll pid:%.2f", roll_pid_out);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    free(pitch_pid);
    free(roll_pid);
    free(mpu);
    vTaskDelete(NULL);
}

//this task will take the inputs by lora tx (radio controller)
void lorarx_task(void* pvParameters){
    mpu6050_t* mpu = (mpu6050_t*)pvParameters;
    uint8_t buf[255];
    while(1){
        TickType_t nowTick = xTaskGetTickCount();
        int send_len = sprintf((char*)buf, "Oi!!%"PRIu32, nowTick);
        lora_send_packet(buf, send_len);
        ESP_LOGI(TAG_LORA, "%d byte packet sent...!", send_len);
        int lost = lora_packet_lost();
        if(lost != 0){
            ESP_LOGW(TAG_LORA, "%d packets lost", lost);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    free(mpu);
    vTaskDelete(NULL);
}

void app_main(void){

    //i2c init
    setup_i2c();

    //mpu init
    mpu6050_t* mpu;
    mpu = setup_mpu();

    //pwm
    esp_err_t ret_pwm = setup_pwm(PWM1_PIN, PWM1_CHANNEL, PWM1_TIMER, 5e3);
    esp_err_t ret_pwm2 = setup_pwm(PWM2_PIN, PWM2_CHANNEL, PWM2_TIMER, 8e3);
    if(ret_pwm != ESP_OK){
        ESP_LOGW(TAG_PWM, "PWM1 init failed!");
        return;
    }
    if(ret_pwm2 != ESP_OK){
        ESP_LOGW(TAG_PWM, "PWM2 init failed!");
        return;
    }

    //adc
    //adc2_config_channel_atten(ADC2_CHANNEL_3, ADC_ATTEN_DB_11);

    //lora init
    //setup_sx1276();

    //xTaskCreatePinnedToCore(pwm_task, "TASK PWM", 2000, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(mpu_task, "TASK MPU6050", 3000, (void*)&mpu, 1, NULL, 1);
    xTaskCreatePinnedToCore(pid_task, "TASK PID", 2500, (void*)&mpu, 1, NULL, 0);
    //xTaskCreatePinnedToCore(lorarx_task, "TASK LORA TX", 2000, (void*)&mpu, 1, NULL, 1);
}


