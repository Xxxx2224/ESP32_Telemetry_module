#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "main";
void calculation_1() {
  volatile uint32_t sum = 0;
  for (uint32_t i = 0; i < 50000000; i++) {
    sum += i;
    if (i % 10000000 == 0) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void calculation_2() {

  volatile uint32_t sum = 0;
  for (uint32_t i = 0; i < 50000000; i++) {
    sum += i;
    if (i % 10000000 == 0) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void app_main() {
  TickType_t start_tick = xTaskGetTickCount();
  calculation_1();
  calculation_2();
  TickType_t end_tick = xTaskGetTickCount();
  uint32_t elapsed_time_ms = (end_tick - start_tick) * portTICK_PERIOD_MS;
  float elapsed_time_s = elapsed_time_ms / 1000.0;

  ESP_LOGI(TAG, "Calculation Task 1 completed in %ld ms (%.2f s)",
           elapsed_time_ms, elapsed_time_s);
}
