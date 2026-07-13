#ifndef LORA_DR_H
#define LORA_DR_H

#include "lora.h"
#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "stdint.h"
#include "freertos/FreeRTOS.h"

#define LORA_CODING_RATE 1
#define LORA_BANDWIDTH 7
#define LORA_SF_RATE 9
#define LORA_FREQUENCY 915e6

const static char* TAG_LORA = "LORA";

typedef unsigned char byte;

void setup_sx1276(void);
void lora_rx_task(void* pvParameters);

#endif
