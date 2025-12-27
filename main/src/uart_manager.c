#include "uart_manager.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mavlink_handler.h"

static const char *TAG = "UART_MANAGER";
#define UART_BUFFER_SIZE 2048
#define UART_PORT_NUM UART_NUM_2

static QueueHandle_t uart_queue;

static void uart_recevier_task(void *arg) {
    uart_event_t event;
    uint8_t *data = (uint8_t *)malloc(1024);
    if (data == NULL) {
        ESP_LOGE(TAG, "Memory allocation failed!");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA: {
                    int length = uart_read_bytes(UART_PORT_NUM, data, 1024, pdMS_TO_TICKS(10));
                    for (int i = 0; i < length; i++) {
                        mavlink_handler_parse_byte(data[i]);
                    }
                    break;
                }
                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "Hardware FIFO Overflow");
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart_queue);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "Ring Buffer Full");
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart_queue);
                    break;
                default:
                    // ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(data);
    vTaskDelete(NULL);
}

void uart_manager_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 57600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, 5, 4, 17, 18)); // TX: 5, RX: 4, RTS: 17, CTS: 18

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUFFER_SIZE,
                                        UART_BUFFER_SIZE, 10, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_set_mode(UART_PORT_NUM, UART_MODE_UART));

    xTaskCreatePinnedToCore(uart_recevier_task, "UART Receiver Task", 4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "UART initialized and task started");
}
