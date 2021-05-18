#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

void app_main() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<LED_BUILTIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    while(1) {
        printf("Hello World!\n");
        gpio_set_level(LED_BUILTIN, 0);
        vTaskDelay(1000 / portTICK_RATE_MS);
        gpio_set_level(LED_BUILTIN, 1);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}