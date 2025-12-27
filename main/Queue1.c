#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#define configMINIMAL_STACK_SIZE 4096

QueueHandle_t myQueue;

void senderTask(void *pvParameters) {
  int dataToSend = 42;

  while (1) {
    if (xQueueSend(myQueue, &dataToSend, pdMS_TO_TICKS(100)) == pdPASS) {
      printf("Veri kuyruğa gönderildi: %d\n", dataToSend);
    } else {
      printf("Kuyruk dolu, veri gönderilemedi.\n");
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void receiverTask(void *pvParameters) {
  int receivedData;

  while (1) {
    if (xQueueReceive(myQueue, &receivedData, pdMS_TO_TICKS(100)) == pdTRUE) {
      printf("Veri alındı: %d\n", receivedData);
    } else {
      printf("Kuyruk boş, veri alınamadı.\n");
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void app_main(void) {

  myQueue = xQueueCreate(10, sizeof(int));
  if (myQueue == NULL) {
    printf("Kuyruk oluşturulamadı!\n");
  }

  xTaskCreate(senderTask, "Sender", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(receiverTask, "Receiver", configMINIMAL_STACK_SIZE, NULL, 1,
              NULL);
}
