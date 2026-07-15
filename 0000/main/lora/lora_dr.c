#include "lora_dr.h"
#include "freertos/idf_additions.h"
#include "lora.h"

QueueHandle_t lora_queue;

void setup_sx1276(){
   if(lora_init() == 0){
       ESP_LOGW(TAG_LORA, "SX1276 init failed!");
       return;
   }

   lora_set_frequency(LORA_FREQUENCY);
   lora_enable_crc();
   lora_set_coding_rate(LORA_CODING_RATE);
   lora_set_bandwidth(LORA_BANDWIDTH);
   lora_set_sync_word(0x12);
   lora_set_spreading_factor(LORA_SF_RATE);
}

void lora_rx_task(void* pvParameters){
    uint8_t buff[2];
    float cmd;
    control_t receive;
    while(1){
        lora_receive();
        if(lora_received()){
            int x = lora_receive_packet(buff, sizeof(buff) - 1);
            if(x > 0){
                cmd = (buff[0] << 8) + buff[1];
                receive.throttle = (uint16_t)(cmd / 4095 * 300);
                xQueueSend(lora_queue, &receive, 0);
            }
        }
        lora_receive();
    } vTaskDelete(NULL);
}

