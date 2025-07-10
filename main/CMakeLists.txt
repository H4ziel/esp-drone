idf_component_register(SRCS "pwm/pwm.c" "main.c" "pid/pid.c"
                    INCLUDE_DIRS "." "pwm" "pid"
                    REQUIRES mpu6050 driver esp_timer)

