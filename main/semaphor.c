#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdio.h>

SemaphoreHandle_t xSemaphore;
int sharedVar = 0;

void task1(void *pvParameters) {
  while (1) {

    printf("Task 1: sharedVar = %d\n", sharedVar);
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
      printf("Task 11: sharedVar = %d\n", sharedVar);
      xSemaphoreGive(xSemaphore);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void task2(void *pvParameters) {
  while (1) {

    printf("Task 2: sharedVar = %d\n", sharedVar);
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
      printf("Task 22: sharedVar = %d\n", sharedVar);
      xSemaphoreGive(xSemaphore);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void app_main(void) {

  xSemaphore = xSemaphoreCreateMutex();

  if (xSemaphore != NULL) {

    xTaskCreatePinnedToCore(task1, "Task1", 2048, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(task2, "Task2", 2048, NULL, 5, NULL, 1);
  } else {
    printf("Semafor oluşturulamadı!\n");
  }
}
