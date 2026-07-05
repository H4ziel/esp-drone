# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS				main.c "pwm/pwm.c" "pid/pid.c" "mpu/mpu.c"
    INCLUDE_DIRS 		"." "pwm" "pid" "mpu"
    PRIV_INCLUDE_DIRS   # optional, add here private include directories
    REQUIRES			esp_driver_ledc esp_driver_gpio esp_timer freertos
    PRIV_REQUIRES       driver
)
