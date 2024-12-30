#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>

SemaphoreHandle_t xSemaphore;  // Semafor handle'ı
int sharedVar = 0;  // Paylaşılan değişken

// Görev 1 (Paylaşılan değişkene yazma)
void task1(void *pvParameters) {
    while (1) {
        // Semafor alınana kadar bekle
        printf("Task 1: sharedVar = %d\n", sharedVar);
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            printf("Task 11: sharedVar = %d\n", sharedVar);  // Paylaşılan değişkeni okuma
            xSemaphoreGive(xSemaphore);  // Semaforu serbest bırak
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1 saniye bekle
    }
}

// Görev 2 (Paylaşılan değişkene okuma)
void task2(void *pvParameters) {
    while (1) {
        // Semafor alınana kadar bekle
        printf("Task 2: sharedVar = %d\n", sharedVar);
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            printf("Task 22: sharedVar = %d\n", sharedVar);  // Paylaşılan değişkeni okuma
            xSemaphoreGive(xSemaphore);  // Semaforu serbest bırak
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1 saniye bekle
    }
}

void app_main(void) {
    // Semaforu oluştur
    xSemaphore = xSemaphoreCreateMutex();  // Mutex semafor oluştur

    if (xSemaphore != NULL) {
          // Semaforu al
        // Görevleri oluştur
        xTaskCreatePinnedToCore(task1, "Task1", 2048, NULL, 10, NULL, 1);
        xTaskCreatePinnedToCore(task2, "Task2", 2048, NULL, 5, NULL, 1);
    } else {
        printf("Semafor oluşturulamadı!\n");
    }
}
