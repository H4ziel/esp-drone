#include <stdint.h>
#include <stdio.h>
#include "lora.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

#define CONFIG_CODING_RATE 1

const static char* TAG_LORA = "LORA";

void setup_sx1276(void);
void lora_tx_task(void* pvParameters);
void adc_init();
int adc_read_pot();

static adc_oneshot_unit_handle_t adc1_handle;

void app_main(){
    adc_init();
    setup_sx1276();

    xTaskCreatePinnedToCore(lora_tx_task, "TASK LORA TX", 2000, NULL, 1, NULL, 1);
}

void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        adc1_handle,
        ADC_CHANNEL_6,
        &chan_config
    ));
}

int adc_read_pot(void)
{
    int raw = 0;

    ESP_ERROR_CHECK(adc_oneshot_read(
        adc1_handle,
        ADC_CHANNEL_6,
        &raw
    ));

    return raw;
}

void setup_sx1276(){
   if(lora_init() == 0){
       ESP_LOGW(TAG_LORA, "SX1276 init failed!");
       return;
   }

   lora_set_frequency(915e6);
   lora_enable_crc();
   lora_set_coding_rate(1);
   lora_set_sync_word(0x12);
   lora_set_bandwidth(7);
   lora_set_spreading_factor(9);
}

//this task will take the inputs by lora tx (radio controller)
void lora_tx_task(void* pvParameters){
    uint8_t msg[2] = {0};
    int cmd = 0;
    while(1){
        for(uint8_t n=0; n<10; n++){
            cmd += adc_read_pot();
        }
        cmd /= 10;
        msg[0] = (cmd & 0xFF00) >> 8;
        msg[1] = (cmd & 0x00FF);
        cmd = 0;
        lora_send_packet(msg, sizeof(msg));
        ESP_LOGI(TAG_LORA, "%d byte packet sent...!", sizeof(msg));
        int lost = lora_packet_lost();
        if(lost != 0){
            ESP_LOGW(TAG_LORA, "%d packets lost", lost);
        }
    }
    vTaskDelete(NULL);
}
