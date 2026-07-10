#include "display.h"
#include "esp_err.h"
#include "hal/i2c_types.h"
#include "i2c_dr.h"
#include "soc/gpio_num.h"
#include "ssd1306.h"

esp_err_t display_init(void){
    ssd1306_config_t display_cfg = {
        .width = SCREEN_WIDTH,
        .height = SCREEN_HEIGHT,
        .fb = NULL,
        .fb_len = 0,
        .bus = SSD1306_I2C,
        .iface.i2c = {
            .addr = DISPLAY_ADDR,
            .port = I2C_NUM,
            .rst_gpio = GPIO_NUM_NC,
        },
    };
    ssd1306_handle_t disp = NULL;
    ESP_ERROR_CHECK(ssd1306_new_i2c(&display_cfg,  &disp));

    return (ssd1306_clear(disp));
}

