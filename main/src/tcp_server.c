#include "tcp_server.h"
#include "data_manager.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TCP_SERVER";

static void socket_listen_task(void *arg) {
    int server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY); // Bind to all interfaces (or use specific IP)
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(8000);

    // Note: The original code bound to 192.168.4.1 explicitly. INADDR_ANY is usually safer/easier.
    // If specific binding is needed:
    // server_addr.sin_addr.s_addr = inet_addr("192.168.4.1");

    if (bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(server_socket);
        vTaskDelete(NULL);
        return;
    }

    if (listen(server_socket, 1) != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        close(server_socket);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Socket listening on port 8000");

    char BUFFER[1024];

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t socklen = sizeof(client_addr);
        int client = accept(server_socket, (struct sockaddr *)&client_addr, &socklen);
        
        if (client < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        ESP_LOGI(TAG, "New connection: IP %s, PORT %d", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

        // Read initial handshake or data if any
        int len = recv(client, BUFFER, sizeof(BUFFER), 0);
        if (len > 0) {
            BUFFER[len] = 0; // Null terminate
            ESP_LOGI(TAG, "Received: %s", BUFFER);
        }

        while (1) {
            char *json_data = data_manager_get_json_string();
            if (json_data == NULL) {
                ESP_LOGE(TAG, "Failed to generate JSON");
                // Wait a bit and retry? Or close?
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            // Send length of JSON + newline? Or just the JSON? Original code sent JSON.
            // Original code sent raw string.
            int to_write = strlen(json_data);
            int written = send(client, json_data, to_write, 0);

            // Free the JSON string immediately
            cJSON_free(json_data); 

            if (written < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(500)); // Update rate 2Hz
        }

        close(client);
        ESP_LOGI(TAG, "Client disconnected");
    }

    close(server_socket);
    vTaskDelete(NULL);
}

void tcp_server_init(void) {
    xTaskCreatePinnedToCore(socket_listen_task, "Socket Listen Task", 4096, NULL, 5, NULL, 1);
}
