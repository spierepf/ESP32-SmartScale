#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

void config_gpio(uint8_t pin, gpio_mode_t mode) {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = mode;
    io_conf.pin_bit_mask = (1ULL<<pin);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

void hx711_init() {
    config_gpio(HX711_PD_SCK, GPIO_MODE_OUTPUT);
    config_gpio(HX711_DOUT, GPIO_MODE_INPUT);
    gpio_set_level(HX711_PD_SCK, 0);
}

int hx711_is_ready() {
    return 0 == gpio_get_level(HX711_DOUT);
}

int32_t hx711_read_value() {
    int32_t value = 0;

    for(int i = 0; i < 25; ++i) {
        gpio_set_level(HX711_PD_SCK, 1);
        ets_delay_us(1);
        value <<= 1;
        value |= (gpio_get_level(HX711_DOUT) & 1);
        gpio_set_level(HX711_PD_SCK, 0);
        ets_delay_us(1);
    }
    value <<= 7;
    value >>= 8;

    return value;
}

void app_main() {
    config_gpio(LED_BUILTIN, GPIO_MODE_OUTPUT);

    hx711_init();

    while(1) {
        printf("Hello World!\n");
        if(hx711_is_ready()) {
            printf("HX711 Reading: %d\n", hx711_read_value());
        }
        gpio_set_level(LED_BUILTIN, 0);
        vTaskDelay(1000 / portTICK_RATE_MS);
        gpio_set_level(LED_BUILTIN, 1);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}