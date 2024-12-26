#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "common/mavlink.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"

#define PASSWORD "Esp32Passss"
#define SSID "ESP32S3"
#define UART_BUFFER_SIZE 2048
static const char *TAG = "wifi softAP";
QueueHandle_t uart_queue;
int client;
uint8_t heartbeat_data[sizeof(mavlink_heartbeat_t) + 4];
void socket_send_client(mavlink_message_t *message){
    if (client >=0 )
    {
        switch (message->msgid)
{
case MAVLINK_MSG_ID_HEARTBEAT:
                mavlink_heartbeat_t heartbeat;
                    mavlink_msg_heartbeat_decode(message, &heartbeat);
                    printf("Heartbeat Tipi: %u\n", heartbeat.type);
                    printf("Heartbeat Otomasyon Seviyesi: %u\n", heartbeat.autopilot);
                    memcpy(heartbeat_data, message->msgid, 24);
                    memcpy(heartbeat_data+4, &heartbeat, sizeof(mavlink_heartbeat_t));
                    send(client,heartbeat_data,sizeof(heartbeat_data),0);
                break;
            
            default:
            printf("Bilinmeyen mesaj: %u\n",message->msgid);
                break;
}
    }
    


}
void socket_listen(void){
 int server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket < 0) {
        perror("Socket oluşturulamadı");
        return;
    }
 struct sockaddr_in server_addr;
 server_addr.sin_addr.s_addr = inet_addr("192.168.1.1");
 server_addr.sin_family = AF_INET;
 server_addr.sin_port = htons(8000);
 if (bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("Socket bağlanamadı");
        close(server_socket);
        return;
    }

 while (1)
 {
    struct sockaddr_in client_addr;
    socklen_t socklen = sizeof(client_addr);
    client = accept(server_socket,(struct sockaddr *)&client_addr,socklen);
    if (client < 0)
    {
        ESP_LOGW(TAG,"Bağlantı kabul edilemedi");
        continue;
    }
    printf("Yeni bağlantı: IP %s, PORT %d\n", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

    char BUFFER[1024];
    int socket_buffer = recv(client,BUFFER,sizeof(BUFFER),0);
    if (socket_buffer > 0)
    {
       printf("alinan veri %s\n",BUFFER);
    } 
    close(client);
    printf("baglanti kapandi");
 }
 
    close(server_socket);

}
void uart_mavlink(uint8_t *data){
    mavlink_message_t message;
    mavlink_status_t status;
    for (size_t i = 0; i < data[1]+12; i++)
    {
        if (mavlink_parse_char(MAVLINK_COMM_0,data[i],&message,&status))
        {
            printf("Mesaj Kimliği: %u\n", message.msgid);
            printf("Sistem ID: %u\n", message.sysid);
            printf("Bileşen ID: %u\n", message.compid);
            socket_send_client(&message);
        }
        
        
    }
    
    
}
void uart_recevier(void)
{
    uart_event_t event;
    uint8_t *data =(u_int8_t *)malloc(1024);
    while (1)
    {
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY) == pdTRUE)
{
   switch (event.type)
   {
   case UART_DATA:
        while (uart_read_bytes(UART_NUM_2, data, 1, pdMS_TO_TICKS(10)) == 1)
        {
            if (data[0] == 0xFD)
            {
                if (uart_read_bytes(UART_NUM_2,data+1,9,portMAX_DELAY) == 5)
                {
                    if (uart_read_bytes(UART_NUM_2, data+10, data[1]+ 2, portMAX_DELAY) == data[1]+ 2)
                    {
                        if (client == 1)
                        {
                            uart_mavlink(data);
                        break;
                        }
                    }
                    
                }
                
            }
            
        }
        
    break;
   
   default:
    break;
   }
}
    }
    free(data);  // Bellek serbest bırakılıyor
    vTaskDelete(NULL); 
}
void uart_init(void){
    const uart_port_t uart_num = UART_NUM_2;
    uart_config_t uart_config = {
        .baud_rate = 57600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh =200,
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num,&uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 4, 5, 17, 18));
    
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2,UART_BUFFER_SIZE,UART_BUFFER_SIZE,10,&uart_queue,0));
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));



}
void wifi_event_handler(void *arg,esp_event_base_t event_base,int32_t event_id,void *event_data)
{
    
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d, reason=%d",
                 MAC2STR(event->mac), event->aid, event->reason);
    }

    
}
void wifi(void){
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    char *ssid_buffer = (char *)malloc(33);
    if (strlen(SSID) == 0) {
        strncpy(ssid_buffer, "ESP32S3" , 33);
        
    } else {
        strncpy(ssid_buffer, SSID , 33);
        
    }
    ESP_LOGW(TAG, "SSID: %s", ssid_buffer);

    
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = SSID,
            .password = PASSWORD,
            .channel = 1,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = 3,
        } 
    };
    
    if (strlen(PASSWORD) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    free(ssid_buffer);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,&wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main(){
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGW(TAG, "ESP_WIFI_MODE_AP");
    wifi();
    uart_init();

}