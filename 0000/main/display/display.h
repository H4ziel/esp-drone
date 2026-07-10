#ifndef DISPLAY_H
#define DISPLAY_H

#include "i2c/i2c_dr.h"
#include "ssd1306.h"

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define DISPLAY_ADDR  0x3C

typedef enum{
	DISPLAY_IDLE = 0,
	DISPLAY_ON = 0xFF
}DISPLAY_STATE;

typedef struct DISPLAY{
	DISPLAY_STATE status;
}display_t;

esp_err_t display_init(void);

#endif
