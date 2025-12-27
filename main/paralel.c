#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "main";
int i = 0;

void calculation_task_1(void *pvParameters) {
  i = 5;

  volatile uint32_t sum = 0;
  for (uint32_t i = 0; i < 50000000; i++) {
    sum += i;
    if (i % 10000000 == 0) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  vTaskDelete(NULL);
}

void calculation_task_2(TickType_t start_tick) {

  volatile uint32_t sum = 0;
  for (uint32_t i = 0; i < 50000000; i++) {
    sum += i;
    if (i % 10000000 == 0) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  TickType_t end_tick = xTaskGetTickCount();
  uint32_t elapsed_time_ms = (end_tick - start_tick) * portTICK_PERIOD_MS;
  float elapsed_time_s = elapsed_time_ms / 1000.0;

  ESP_LOGI(TAG, "Calculation Task 2 completed in %ld ms (%.2f s)",
           elapsed_time_ms, elapsed_time_s);
  printf("i degeri: %d\n", i);

  vTaskDelete(NULL);
}

void app_main() {
  TickType_t start_tick = xTaskGetTickCount();
  xTaskCreatePinnedToCore(calculation_task_1, "Calculation Task 1", 4096, NULL,
                          5, NULL, 1);
  xTaskCreatePinnedToCore(calculation_task_2, "Calculation Task 2", 4096,
                          start_tick, 5, NULL, 1);
}
