#include "pwm.h"
#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "hal/ledc_types.h"
#include "pid.h"

esp_err_t setup_pwm(gpio_num_t PWM_PIN, ledc_timer_t PWM_TIMER, 
									 ledc_channel_t PWM_CHANNEL, uint32_t freq){

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

static uint32_t motor_cmd_to_duty(uint16_t cmd){
    return ((uint32_t)clamp_float((float)cmd , 0.0f, (float)PWM_MAX_DUTY) * 
												  PWM_MAX_DUTY) / MOTOR_CMD_MAX;
}

void motor_set_cmd(uint16_t cmd, ledc_channel_t channel){
    uint32_t duty = motor_cmd_to_duty(cmd);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
}

void pwm_task(void* pvParameters){
    QueueHandle_t output_queue_handle = (QueueHandle_t)pvParameters;

    control_t pid_received;

    while (true) {
        if (xQueueReceive(output_queue_handle, &pid_received, portMAX_DELAY) == 
																	    pdPASS){

            motor_set_cmd(pid_received.out[MOTOR_FL], PWM2_CHANNEL);
            motor_set_cmd(pid_received.out[MOTOR_FR], PWM1_CHANNEL);

            ESP_LOGI(TAG_PWM, "FL cmd: %d", pid_received.out[MOTOR_FL]);
            ESP_LOGI(TAG_PWM, "FR cmd: %d", pid_received.out[MOTOR_FR]);
			
			vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}
