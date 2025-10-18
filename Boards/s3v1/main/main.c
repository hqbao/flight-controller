#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "main.c"

TaskHandle_t task_hangle_1 = NULL;
TaskHandle_t task_hangle_2 = NULL;

void delay(uint32_t ms) {
  vTaskDelay(pdMS_TO_TICKS(ms));
}

void core0() {
  while (1) { delay(1000); }
}

void core1() {
  while (1) { delay(1000); }
}

void app_main(void) {
  xTaskCreatePinnedToCore(core0, "Core 0", 4096, NULL, 10, &task_hangle_1, 0);
  xTaskCreatePinnedToCore(core1, "Core 1", 4096, NULL, 10, &task_hangle_2, 1);
  while (1) { delay(1000); }
}
