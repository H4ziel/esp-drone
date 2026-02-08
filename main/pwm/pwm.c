#include "pwm.h"
#include "esp_err.h"
#include "hal/ledc_types.h"

esp_err_t setup_pwm(gpio_num_t PWM_PIN, ledc_timer_t PWM_TIMER, ledc_channel_t PWM_CHANNEL, uint32_t freq) {

	uint32_t max_freq_config = 80e6 / (1 << PWM_DUTY_RESOLUTION);

	if(freq > max_freq_config){
		return ESP_FAIL;
	}

    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
        .duty_resolution = PWM_DUTY_RESOLUTION,
        .freq_hz = freq
    };

    esp_err_t err = ledc_timer_config(&timer_config);
    if (err != ESP_OK) {
        return err;
    }

    ledc_channel_config_t channel_config = {
        .channel = PWM_CHANNEL,
        .gpio_num = PWM_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };

    return ledc_channel_config(&channel_config);
}

void duty_att(uint16_t dc, ledc_channel_t PWM_CHANNEL){
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL, dc);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL);
}
