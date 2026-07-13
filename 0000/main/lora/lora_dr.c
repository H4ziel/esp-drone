#include "lora_dr.h"
#include "lora.h"

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
    uint8_t buff[255];
    while(1){
        lora_receive();
        if(lora_received()){
            int x = lora_receive_packet(buff, sizeof(buff) - 1);
            if(x > 0){
                buff[x] = '\0';
                ESP_LOGI(TAG_LORA, "%s", (char*)buff);
            }
        }
        lora_receive();
    vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL);
}
