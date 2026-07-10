# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS				main.c "pwm/pwm.c" "pid/pid.c" "mpu/mpu.c" "i2c/i2c_dr.c" "display/display.c"
    INCLUDE_DIRS 		"." "pwm" "pid" "mpu" "display" "i2c"
    PRIV_INCLUDE_DIRS   # optional, add here private include directories
    REQUIRES		    esp_driver_i2c esp_driver_ledc esp_driver_gpio esp_timer freertos ssd1306
    PRIV_REQUIRES       driver
)
