#ifndef PWM_H
#define PWM_H

#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define PWM_DUTY_RESOLUTION 12

void duty_att(uint16_t dc, ledc_timer_t PWM_TIMER, ledc_channel_t PWM_CHANNEL);
esp_err_t setup_pwm(gpio_num_t PWM_PIN, ledc_timer_t PWM_TIMER, ledc_channel_t
                                               PWM_CHANNEL, uint32_t frequency);

#endif
