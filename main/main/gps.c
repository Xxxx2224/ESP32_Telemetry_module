#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <ctype.h>

#define EX_UART_NUM UART_NUM_2
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static const char *TAG = "uart_events";
#define PATTERN_CHR_NUM (3)

extern "C" {
    void uart_event_task(void *pvParameters);
    void app_main(); // app_main fonksiyonunu C bağlantı sembolü olarak tanımlayın
}

static QueueHandle_t uart0_queue;
TinyGPS gps; // TinyGPS nesnesi oluşturun

void uart_event_task(void *pvParameters) {
    uart_event_t event;
    uint8_t *dtmp = (uint8_t *) malloc(RD_BUF_SIZE);
    for(;;) {
        // Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
                // Event of UART receiving data
                case UART_DATA:
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    ESP_LOGI(TAG, "[DATA EVT]:");
                    for (int i = 0; i < event.size; i++) {
                        printf("Received byte: 0x%02X (%c)\n", dtmp[i], isprint(dtmp[i]) ? dtmp[i] : '.');
                        gps.encode(dtmp[i]); // TinyGPS ile veriyi işleyin
                    }
                    printf("\n");

                    // GPS verilerini kontrol edin ve loglayın
                    float latitude, longitude;
                    gps.f_get_position(&latitude, &longitude);
                    if (latitude != TinyGPS::GPS_INVALID_F_ANGLE && longitude != TinyGPS::GPS_INVALID_F_ANGLE) {
                        ESP_LOGI(TAG, "Latitude: %f", latitude);
                        ESP_LOGI(TAG, "Longitude: %f", longitude);
                        ESP_LOGI(TAG, "Altitude: %f", gps.f_altitude());
                        ESP_LOGI(TAG, "Satellites: %d", gps.satellites());
                        ESP_LOGI(TAG, "HDOP: %f", (double)gps.hdop());
                    } else {
                        ESP_LOGI(TAG, "GPS location not updated");
                    }
                    break;
                // Event of UART pattern detection
                case UART_PATTERN_DET:
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED]");
                    break;
                // Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    vTaskDelete(NULL);
}

extern "C" void app_main() {
    esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 5, 4, 17, 18));

    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(EX_UART_NUM, 20);

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 10, NULL);
}