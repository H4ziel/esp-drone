#include "pwm.h"

esp_err_t setup_pwm(gpio_num_t PWM_PIN, ledc_timer_t PWM_TIMER, ledc_channel_t
                                                    PWM_CHANNEL, uint32_t freq){
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_APB_CLK,
        .duty_resolution = PWM_DUTY_RESOLUTION,
        .freq_hz = freq
    };
    ledc_timer_config(&timer_config);

    ledc_channel_config_t channel_config = {
        .channel = PWM_CHANNEL,
        .gpio_num = PWM_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = PWM_TIMER
    };

    return ledc_channel_config(&channel_config);
}

void duty_att(uint16_t dc, ledc_timer_t PWM_TIMER, ledc_channel_t PWM_CHANNEL){
    if(dc > pow(2, PWM_DUTY_RESOLUTION)){
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL, 0);
    }
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL, dc);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL);
}
