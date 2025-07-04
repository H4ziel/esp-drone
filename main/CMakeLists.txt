idf_component_register(SRCS "pwm/pwm.c" "main.c"
                    INCLUDE_DIRS "." "pwm"
                    REQUIRES mpu6050 driver esp_timer)

