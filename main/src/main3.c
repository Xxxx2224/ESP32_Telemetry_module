#include "wifi_manager.h"
#include "uart_manager.h"
#include "mavlink_handler.h"
#include "data_manager.h"
#include "tcp_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "APP_MAIN";

void app_main(void) {
    ESP_LOGI(TAG, "Starting ESP32 Telemetry Module...");

    // 1. Initialize Data Manager (Mutexes, Shared State)
    data_manager_init();

    // 2. Initialize Mavlink Handler
    mavlink_handler_init();

    // 3. Initialize WiFi (SoftAP)
    wifi_manager_init();

    // 4. Initialize UART (Starts receiving task)
    uart_manager_init();

    // 5. Initialize TCP Server (Starts listening task)
    tcp_server_init();

    ESP_LOGI(TAG, "All modules initialized.");

    // Main task can be deleted or used for monitoring
    // vTaskDelete(NULL); 
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "System running...");
        // Optional: Print some stats or free memory
        // ESP_LOGI(TAG, "Free Heap: %d", esp_get_free_heap_size());
    }
}
