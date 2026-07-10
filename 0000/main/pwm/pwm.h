#ifndef PWM_H
#define PWM_H

#include <stdbool.h>

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define PWM_DUTY_RESOLUTION 10
#define PWM1_PIN 			GPIO_NUM_2
#define PWM1_TIMER 			LEDC_TIMER_1
#define PWM1_CHANNEL 		LEDC_CHANNEL_0
#define PWM_BASE_DUTY		(uint16_t)(((1 << PWM_DUTY_RESOLUTION) - 1) / \
															PWM_DUTY_RESOLUTION)
#define MOTOR_CMD_MIN 		0
#define MOTOR_CMD_MAX		1000
#define PWM_MAX_DUTY     	((1 << PWM_DUTY_RESOLUTION) - 1)
#define PWM_FREQ_HZ      	20e3
#define PWM2_PIN 	 		GPIO_NUM_4
#define PWM2_TIMER   		LEDC_TIMER_0
#define PWM2_CHANNEL 		LEDC_CHANNEL_1

typedef enum{
    MOTOR_FL = 0,
    MOTOR_FR,
    MOTOR_BL,
    MOTOR_BR,
    N_MOTORs
}MOTOR_ENUM;

typedef struct{
    uint16_t out[N_MOTORs];
    uint16_t throttle;
}control_t;

const static char* TAG_PWM = "PWM";

esp_err_t setup_pwm(gpio_num_t PWM_PIN, ledc_timer_t PWM_TIMER, ledc_channel_t
                                               PWM_CHANNEL, uint32_t frequency);
static uint32_t motor_cmd_to_duty(uint16_t cmd);
void motor_set_cmd(uint16_t cmd, ledc_channel_t channel);
void pwm_task(void* pvParameters);

#endif
