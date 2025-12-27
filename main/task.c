#include "freertos/task.h"
#include "freertos/FreeRTOS.h"
#include <stdio.h>
#define configMINIMAL_STACK_SIZE 4096

TaskHandle_t task1Handle, task2Handle, task3Handle;

void task1(void *pvParameters) {
  while (1) {
    printf("Görev 1 çalışıyor.\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void task2(void *xTask1Handle) {
  TaskHandle_t xTask1Handle = (TaskHandle_t)xTask1Handle;
  while (1) {

    if (task1Handle != NULL) {
      printf("Görev 1 askıya alınıyor.\n");
      vTaskSuspend(task1Handle);
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    if (task1Handle != NULL) {
      printf("Görev 1 devam ettiriliyor.\n");
      vTaskResume(task1Handle);
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void task3(void *xTask3Handle) {
  TaskHandle_t xTask3Handle = (TaskHandle_t)xTask3Handle;
  UBaseType_t priority;

  while (1) {

    priority = uxTaskPriorityGet(task3Handle);
    printf("Görev 3 çalışıyor, mevcut öncelik: %d\n", (int)priority);

    if (priority < configMAX_PRIORITIES - 1) {
      vTaskPrioritySet(task3Handle, priority + 1);
      printf("Görev 3 önceliği artırıldı: %d\n", (int)(priority + 1));
    }

    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

int main(void) {

  xTaskCreatePinnedToCore(task1, "Task1", configMINIMAL_STACK_SIZE, NULL, 1,
                          &task1Handle, 1);
  xTaskCreatePinnedToCore(task2, "Task2", configMINIMAL_STACK_SIZE,
                          (void *)task1Handle, 2, &task2Handle, 1);
  xTaskCreatePinnedToCore(task3, "Task3", configMINIMAL_STACK_SIZE,
                          (void *)task3Handle, 3, &task3Handle,
                          1); // Görev 3 tanıtıcı ile oluşturuldu
  return 0;
}
