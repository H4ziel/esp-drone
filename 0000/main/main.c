#include <stddef.h>
#include <stdint.h>
#include "freertos/idf_additions.h"
#include "mpu/mpu.h"
#include "portmacro.h"
#include "pwm/pwm.h"

#include "esp_log.h"
#include "esp_err.h"
#include "pid/pid.h"

typedef unsigned char byte;

extern volatile bool calibration;
extern QueueHandle_t mpu_queue;
extern QueueHandle_t pid_queue;

void app_main(void){

    mpu_queue = xQueueCreate(5, sizeof(mpu6050_t));
    pid_queue = xQueueCreate(1, sizeof(control_t));

    if(mpu_queue == NULL)
        ESP_LOGE(TAG_MPU, "MPU QUEUE FAIL");

    if(pid_queue == NULL)
        ESP_LOGE(TAG_PID, "OUTPUT QUEUE FAIL");

    //i2c init
    setup_i2c();

    //mpu init
    mpu6050_t mpu;
    setup_mpu();
    mpu.offset_pitch = 0;
    mpu.offset_roll = 0;

    //pwm
    esp_err_t ret_pwm = setup_pwm(PWM1_PIN, PWM1_CHANNEL, PWM1_TIMER, 
																   PWM_FREQ_HZ);
    esp_err_t ret_pwm2 = setup_pwm(PWM1_PIN, PWM2_TIMER, PWM2_CHANNEL, 
																   PWM_FREQ_HZ);
    if(ret_pwm != ESP_OK){
        ESP_LOGW(TAG_PWM, "PWM1 init failed!");
        return;
    }
    if(ret_pwm2 != ESP_OK){
        ESP_LOGW(TAG_PWM, "PWM2 init failed!");
        return;
    }
	
    xTaskCreatePinnedToCore(pwm_task, "TASK PWM", 2000, (void*)pid_queue, 1, 
																	   NULL, 0);
    xTaskCreatePinnedToCore(mpu_task, "TASK MPU6050", 3000, (void*)&mpu, 1, 
																	   NULL, 1);
    xTaskCreatePinnedToCore(pid_task, "TASK PID", 2500, (void*)mpu_queue, 1, 
																	   NULL, 0);
}


