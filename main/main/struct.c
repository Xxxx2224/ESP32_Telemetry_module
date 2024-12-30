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
#include "cJSON.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"


typedef struct {
    int id;
    float temperature;
    int speed;
    char message[100];
} MavlinkData;
cJSON* mavlinkDataToJSON(MavlinkData* data) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "id", data->id);
    cJSON_AddNumberToObject(root, "temperature", data->temperature);
    cJSON_AddNumberToObject(root, "speed", data->speed);
    cJSON_AddStringToObject(root, "message", data->message);
    return root;
}
void JsonUpdateNumber(cJSON *root,char *key,double value){
    cJSON_ReplaceItemInObject(root,key, cJSON_CreateNumber(value));
}
void JsonUpdateString(cJSON *root,char *key,char* value){
    cJSON_ReplaceItemInObject(root,key, cJSON_CreateString(value));
}

int app_main()
{
    MavlinkData data = {0};
    cJSON *root = mavlinkDataToJSON(&data);
    char* json_data;
    json_data = cJSON_PrintUnformatted(root);
    json_data = cJSON_PrintUnformatted(root);
    /*int sent_bytes = send(client_sock, json_data, strlen(json_data), 0);
        if (sent_bytes < 0) {
            perror("Veri gönderim hatası");
            break;
        }

        printf("Gönderilen veri: %s\n", json_data);
        free(json_data);*/

}
