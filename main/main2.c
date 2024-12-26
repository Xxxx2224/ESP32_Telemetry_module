#include <string.h>
#include <stdio.h>
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

#define PASSWORD "Esp32Passss"
#define SSID "ESP32S3"
#define UART_BUFFER_SIZE 2048
static const char *TAG = "wifi softAP";
mavlink_message_t msg;
mavlink_status_t status;
QueueHandle_t uart_queue;
QueueHandle_t cJsonHandle;

typedef struct {
    int id;
    char message[100];
    int custom_mode;
    int type;
    int speed;
    int autopilot;
    int base_mode;
    int system_status;
    int mavlink_version;
    int battery_voltage;
    int battery_remaining;
    long lat;
    long lon;
    long alt;
    int fix_type;
    float roll;
    float pitch;
    float yaw;
    int satellites_visible;
    long global_lat;
    long global_lon;
    long global_alt;
    int vcc;
    int vservo;
    int flags;
    int target_bearing;
    float xtrack_error;
    int mission_seq;
    float airspeed;
    float altitude;
    int xacc;
    int yacc;
    int zacc;
    int xgyro;
    int ygyro;
    int zgyro;
    float pressure;
    int temperature;
    int command;
    float param1;
    float param2;
    int severity;
    char status_message[50];
    float vibration_x;
    float vibration_y;
    float vibration_z;
    long clipping_0;
    long clipping_1;
    long clipping_2;
    int servo1_raw;
    int servo2_raw;
    int servo3_raw;
    int servo4_raw;
    int chan1_raw;
    int chan2_raw;
    int chan3_raw;
    int chan4_raw;
    long terrain_lat;
    long terrain_lon;
    float terrain_height;
} MavlinkData;

cJSON* mavlinkDataToJSON(MavlinkData* data) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "id", data->id);
    cJSON_AddNumberToObject(root, "temperature", data->temperature);
    cJSON_AddNumberToObject(root, "speed", data->speed);
    cJSON_AddStringToObject(root, "message", data->message);
    cJSON_AddNumberToObject(root, "custom_mode", data->custom_mode);
    cJSON_AddNumberToObject(root, "type", data->type);
    cJSON_AddNumberToObject(root, "autopilot", data->autopilot);
    cJSON_AddNumberToObject(root, "base_mode", data->base_mode);
    cJSON_AddNumberToObject(root, "system_status", data->system_status);
    cJSON_AddNumberToObject(root, "mavlink_version", data->mavlink_version);
    cJSON_AddNumberToObject(root, "battery_voltage", data->battery_voltage);
    cJSON_AddNumberToObject(root, "battery_remaining", data->battery_remaining);
    cJSON_AddNumberToObject(root, "lat", data->lat);
    cJSON_AddNumberToObject(root, "lon", data->lon);
    cJSON_AddNumberToObject(root, "alt", data->alt);
    cJSON_AddNumberToObject(root, "fix_type", data->fix_type);
    cJSON_AddNumberToObject(root, "roll", data->roll);
    cJSON_AddNumberToObject(root, "pitch", data->pitch);
    cJSON_AddNumberToObject(root, "yaw", data->yaw);
    cJSON_AddNumberToObject(root, "satellites_visible", data->satellites_visible);
    cJSON_AddNumberToObject(root, "global_lat", data->global_lat);
    cJSON_AddNumberToObject(root, "global_lon", data->global_lon);
    cJSON_AddNumberToObject(root, "global_alt", data->global_alt);
    cJSON_AddNumberToObject(root, "vcc", data->vcc);
    cJSON_AddNumberToObject(root, "vservo", data->vservo);
    cJSON_AddNumberToObject(root, "flags", data->flags);
    cJSON_AddNumberToObject(root, "target_bearing", data->target_bearing);
    cJSON_AddNumberToObject(root, "xtrack_error", data->xtrack_error);
    cJSON_AddNumberToObject(root, "mission_seq", data->mission_seq);
    cJSON_AddNumberToObject(root, "airspeed", data->airspeed);
    cJSON_AddNumberToObject(root, "altitude", data->altitude);
    cJSON_AddNumberToObject(root, "xacc", data->xacc);
    cJSON_AddNumberToObject(root, "yacc", data->yacc);
    cJSON_AddNumberToObject(root, "zacc", data->zacc);
    cJSON_AddNumberToObject(root, "xgyro", data->xgyro);
    cJSON_AddNumberToObject(root, "ygyro", data->ygyro);
    cJSON_AddNumberToObject(root, "zgyro", data->zgyro);
    cJSON_AddNumberToObject(root, "pressure", data->pressure);
    cJSON_AddNumberToObject(root, "command", data->command);
    cJSON_AddNumberToObject(root, "param1", data->param1);
    cJSON_AddNumberToObject(root, "param2", data->param2);
    cJSON_AddNumberToObject(root, "severity", data->severity);
    cJSON_AddStringToObject(root, "status_message", data->status_message);
    cJSON_AddNumberToObject(root, "vibration_x", data->vibration_x);
    cJSON_AddNumberToObject(root, "vibration_y", data->vibration_y);
    cJSON_AddNumberToObject(root, "vibration_z", data->vibration_z);
    cJSON_AddNumberToObject(root, "clipping_0", data->clipping_0);
    cJSON_AddNumberToObject(root, "clipping_1", data->clipping_1);
    cJSON_AddNumberToObject(root, "clipping_2", data->clipping_2);
    cJSON_AddNumberToObject(root, "servo1_raw", data->servo1_raw);
    cJSON_AddNumberToObject(root, "servo2_raw", data->servo2_raw);
    cJSON_AddNumberToObject(root, "servo3_raw", data->servo3_raw);
    cJSON_AddNumberToObject(root, "servo4_raw", data->servo4_raw);
    cJSON_AddNumberToObject(root, "chan1_raw", data->chan1_raw);
    cJSON_AddNumberToObject(root, "chan2_raw", data->chan2_raw);
    cJSON_AddNumberToObject(root, "chan3_raw", data->chan3_raw);
    cJSON_AddNumberToObject(root, "chan4_raw", data->chan4_raw);
    cJSON_AddNumberToObject(root, "terrain_lat", data->terrain_lat);
    cJSON_AddNumberToObject(root, "terrain_lon", data->terrain_lon);
    cJSON_AddNumberToObject(root, "terrain_height", data->terrain_height);
    return root;
}
void JsonUpdateNumber(cJSON *root,char *key,double value){
    cJSON_ReplaceItemInObject(root,key, cJSON_CreateNumber(value));
}
void JsonUpdateString(cJSON *root,char *key,char* value){
    cJSON_ReplaceItemInObject(root,key, cJSON_CreateString(value));
}
void socket_listen(void){
    MavlinkData data = {0};
    cJSON *root = mavlinkDataToJSON(&data);
    BaseType_t result = xQueueSend(cJsonHandle,root,portMAX_DELAY);
    if (result != pdTRUE) {
    printf("Veri kuyruğa gönderilemedi!\n");
    if (root != NULL) {
            cJSON_Delete(root); // root nesnesini serbest bırak
        } 
    return;
    }
    int server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket < 0) {
        perror("Socket oluşturulamadı");
        if (root != NULL) {
            cJSON_Delete(root); // root nesnesini serbest bırak
        } // root nesnesini serbest bırak
        return;
    }
    struct sockaddr_in server_addr;
    server_addr.sin_addr.s_addr = inet_addr("192.168.1.1");
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(8000);
    if (bind(server_socket,(struct sockaddr *)&server_addr,sizeof(server_addr)) < 0) 
    {
        ESP_LOGW(TAG,"Socket baglanamadi");
        close(server_socket);
        if (root != NULL) {
            cJSON_Delete(root); // root nesnesini serbest bırak
        }
    };
 
    struct sockaddr_in client_addr;
    socklen_t socklen = sizeof(client_addr);
    int client = accept(server_socket,(struct sockaddr *)&client_addr,&socklen);
    while (client < 0)
    {
        ESP_LOGW(TAG,"Bağlantı kabul edilemedi");
        client = accept(server_socket,(struct sockaddr *)&client_addr,&socklen);
    }
    printf("Yeni bağlantı: IP %s, PORT %d\n", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
    
    char* json_data;
    json_data = cJSON_PrintUnformatted(root);
    char BUFFER[1024];
    while (1)
 {
    int socket_buffer = recv(client, BUFFER, sizeof(BUFFER), 0);
    if (socket_buffer < 0) {
        perror("recv failed");
        break;
    } else if (socket_buffer == 0) {
        int client1 = accept(server_socket,(struct sockaddr *)&client_addr,&socklen);
        while (client1 < 0)
    {
        ESP_LOGW(TAG,"Bağlantı kabul edilemedi");
        vTaskDelay(pdMS_TO_TICKS(100)); // Add a delay to prevent busy-wait loop
        client1 = accept(server_socket,(struct sockaddr *)&client_addr,&socklen);
    }
        printf("Yeni bağlantı: IP %s, PORT %d\n", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
        continue;
    }
    if (socket_buffer > 0)
    {
       printf("alinan veri %s\n",BUFFER);

    } 

    json_data = cJSON_PrintUnformatted(root);
    send(client,json_data,strlen(json_data),0);
    vTaskDelay(pdMS_TO_TICKS(150));
}
    if (json_data != NULL) {
        free(json_data);
     }
    close(client);
    printf("baglanti kapandi");
    close(server_socket); // server_socket'i kapat
    if (root != NULL) {
            cJSON_Delete(root); // root nesnesini serbest bırak
        }// root nesnesini serbest bırak
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
            switch (message.msgid)
            {
            case MAVLINK_MSG_ID_HEARTBEAT:
                mavlink_heartbeat_t heartbeat;
                mavlink_msg_heartbeat_decode(&message, &heartbeat);
                 break;
            
            default:
            printf("Bilinmeyen mesaj: %u\n", message.msgid);
                break;
            }
        }
        
        
    }
    
    
}
void print_esp32_ap_ip() {
    esp_netif_ip_info_t ip_info;
    esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"); // AP modunun netif'ini al

    if (ap_netif == NULL) {
        printf("AP ağ arayüzü bulunamadı!\n");
        return;
    }

    esp_netif_get_ip_info(ap_netif, &ip_info); // IP bilgilerini al

    printf("ESP32'nin IP Adresi: %s\n", ip4addr_ntoa(&ip_info.ip));
    printf("Ağ Maskesi: %s\n", ip4addr_ntoa(&ip_info.netmask));
    printf("Varsayılan Ağ Geçidi: %s\n", ip4addr_ntoa(&ip_info.gw));
}
void send_request_data_stream(void)
{
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    // Request Data Stream mesajını oluştur
    mavlink_msg_request_data_stream_pack(
        1,        // Sistem ID
        1,     // Komponent ID
        &msg,
        1,        // Hedef Sistem ID
        1,     // Hedef Komponent ID
        MAV_DATA_STREAM_ALL,  // Tüm veri akışları
        120,                   // İstek frekansı (Hz)
        1                     // Akışı etkinleştir (1: Etkin, 0: Devre Dışı)
    );

    // Mesajı buffer'a yaz
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    // UART'a gönder
    uart_write_bytes(UART_NUM_2, (const char *)buffer, len);
}
void handleMavlinkMessage(cJSON *root, const mavlink_message_t *msg) {
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t heartbeat;
        mavlink_msg_heartbeat_decode(msg, &heartbeat);
        printf("Heartbeat - custom_mode: %ld\n", heartbeat.custom_mode);
        printf("Heartbeat - type: %d\n", heartbeat.type);
        printf("Heartbeat - autopilot: %d\n", heartbeat.autopilot);
        printf("Heartbeat - base_mode: %d\n", heartbeat.base_mode);
        printf("Heartbeat - system_status: %d\n", heartbeat.system_status);
        printf("Heartbeat - mavlink_version: %d\n", heartbeat.mavlink_version);
        JsonUpdateNumber(root, "custom_mode", heartbeat.custom_mode);
        JsonUpdateNumber(root, "type", heartbeat.type);
        JsonUpdateNumber(root, "autopilot", heartbeat.autopilot);
        JsonUpdateNumber(root, "base_mode", heartbeat.base_mode);
        JsonUpdateNumber(root, "system_status", heartbeat.system_status);
        JsonUpdateNumber(root, "mavlink_version", heartbeat.mavlink_version);
        break;
    }

    case MAVLINK_MSG_ID_SYS_STATUS: {
        mavlink_sys_status_t sys_status;
        mavlink_msg_sys_status_decode(msg, &sys_status);
        printf("System Status - Battery Voltage: %d, Battery Remaining: %d%%\n",
               sys_status.voltage_battery, sys_status.battery_remaining);
        JsonUpdateNumber(root, "battery_voltage", sys_status.voltage_battery);
        JsonUpdateNumber(root, "battery_remaining", sys_status.battery_remaining);
        break;
    }

    case MAVLINK_MSG_ID_GPS_RAW_INT: {
        mavlink_gps_raw_int_t gps_raw;
        mavlink_msg_gps_raw_int_decode(msg, &gps_raw);
        printf("GPS - Lat: %ld, Lon: %ld, Alt: %ld, Fix: %d\n", gps_raw.lat, gps_raw.lon, gps_raw.alt, gps_raw.fix_type);
        JsonUpdateNumber(root, "lat", gps_raw.lat);
        JsonUpdateNumber(root, "lon", gps_raw.lon);
        JsonUpdateNumber(root, "alt", gps_raw.alt);
        JsonUpdateNumber(root, "fix_type", gps_raw.fix_type);
        break;
    }

    case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(msg, &attitude);
        printf("Attitude - Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", attitude.roll, attitude.pitch, attitude.yaw);
        JsonUpdateNumber(root, "roll", attitude.roll);
        JsonUpdateNumber(root, "pitch", attitude.pitch);
        JsonUpdateNumber(root, "yaw", attitude.yaw);
        break;
    }

    case MAVLINK_MSG_ID_GPS_STATUS: {
        mavlink_gps_status_t gps_status;
        mavlink_msg_gps_status_decode(msg, &gps_status);
        printf("GPS Status - Satellites Visible: %d\n", gps_status.satellites_visible);
        JsonUpdateNumber(root, "satellites_visible", gps_status.satellites_visible);
        break;
    }

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t global_pos;
        mavlink_msg_global_position_int_decode(msg, &global_pos);
        printf("Global Position - Lat: %ld, Lon: %ld, Alt: %ld\n",
               global_pos.lat, global_pos.lon, global_pos.alt);
        JsonUpdateNumber(root, "global_lat", global_pos.lat);
        JsonUpdateNumber(root, "global_lon", global_pos.lon);
        JsonUpdateNumber(root, "global_alt", global_pos.alt);
        break;
    }

    case MAVLINK_MSG_ID_POWER_STATUS: {
        mavlink_power_status_t power_status;
        mavlink_msg_power_status_decode(msg, &power_status);
        printf("Power Status - Vcc: %d, Vservo: %d, Flags: %d\n",
               power_status.Vcc, power_status.Vservo, power_status.flags);
        JsonUpdateNumber(root, "vcc", power_status.Vcc);
        JsonUpdateNumber(root, "vservo", power_status.Vservo);
        JsonUpdateNumber(root, "flags", power_status.flags);
        break;
    }

    case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: {
        mavlink_nav_controller_output_t nav_output;
        mavlink_msg_nav_controller_output_decode(msg, &nav_output);
        printf("Nav Controller - Target Bearing: %d, Xtrack Error: %.2f\n",
               nav_output.target_bearing, nav_output.xtrack_error);
        JsonUpdateNumber(root, "target_bearing", nav_output.target_bearing);
        JsonUpdateNumber(root, "xtrack_error", nav_output.xtrack_error);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_CURRENT: {
        mavlink_mission_current_t mission_current;
        mavlink_msg_mission_current_decode(msg, &mission_current);
        printf("Mission Current - Sequence: %d\n", mission_current.seq);
        JsonUpdateNumber(root, "mission_seq", mission_current.seq);
        break;
    }

    case MAVLINK_MSG_ID_VFR_HUD: {
        mavlink_vfr_hud_t vfr_hud;
        mavlink_msg_vfr_hud_decode(msg, &vfr_hud);
        printf("VFR HUD - Airspeed: %.2f, Altitude: %.2f\n",
               vfr_hud.airspeed, vfr_hud.alt);
        JsonUpdateNumber(root, "airspeed", vfr_hud.airspeed);
        JsonUpdateNumber(root, "altitude", vfr_hud.alt);
        break;
    }

    case MAVLINK_MSG_ID_RAW_IMU: {
        mavlink_raw_imu_t raw_imu;
        mavlink_msg_raw_imu_decode(msg, &raw_imu);
        printf("Raw IMU - Accel: [%d, %d, %d], Gyro: [%d, %d, %d]\n",
               raw_imu.xacc, raw_imu.yacc, raw_imu.zacc,
               raw_imu.xgyro, raw_imu.ygyro, raw_imu.zgyro);
        JsonUpdateNumber(root, "xacc", raw_imu.xacc);
        JsonUpdateNumber(root, "yacc", raw_imu.yacc);
        JsonUpdateNumber(root, "zacc", raw_imu.zacc);
        JsonUpdateNumber(root, "xgyro", raw_imu.xgyro);
        JsonUpdateNumber(root, "ygyro", raw_imu.ygyro);
        JsonUpdateNumber(root, "zgyro", raw_imu.zgyro);
        break;
    }

    case MAVLINK_MSG_ID_SCALED_PRESSURE: {
        mavlink_scaled_pressure_t scaled_pressure;
        mavlink_msg_scaled_pressure_decode(msg, &scaled_pressure);
        printf("Scaled Pressure - Pressure: %.2f, Temperature: %d\n",
               scaled_pressure.press_abs, scaled_pressure.temperature);
        JsonUpdateNumber(root, "pressure", scaled_pressure.press_abs);
        JsonUpdateNumber(root, "temperature", scaled_pressure.temperature);
        break;
    }

    case MAVLINK_MSG_ID_COMMAND_LONG: {
        mavlink_command_long_t command;
        mavlink_msg_command_long_decode(msg, &command);
        printf("Command Long - Command: %d, Param1: %.2f, Param2: %.2f\n", 
               command.command, command.param1, command.param2);
        JsonUpdateNumber(root, "command", command.command);
        JsonUpdateNumber(root, "param1", command.param1);
        JsonUpdateNumber(root, "param2", command.param2);
        break;
    }

    case MAVLINK_MSG_ID_STATUSTEXT: {
        mavlink_statustext_t status_text;
        mavlink_msg_statustext_decode(msg, &status_text);
        printf("Status Text - Severity: %d, Message: %s\n", status_text.severity, status_text.text);
        JsonUpdateNumber(root, "severity", status_text.severity);
        JsonUpdateString(root, "message", status_text.text);
        break;
    }

    case MAVLINK_MSG_ID_VIBRATION: {
        mavlink_vibration_t vibration;
        mavlink_msg_vibration_decode(msg, &vibration);
        printf("Vibration Message Received:  Vibration X: %.2f, Vibration Y: %.2f, Vibration Z: %.2f, Clipping 0: %ld, Clipping 1: %ld, Clipping 2: %ld\n",
               vibration.vibration_x, vibration.vibration_y, vibration.vibration_z,
               vibration.clipping_0, vibration.clipping_1, vibration.clipping_2);
        JsonUpdateNumber(root, "vibration_x", vibration.vibration_x);
        JsonUpdateNumber(root, "vibration_y", vibration.vibration_y);
        JsonUpdateNumber(root, "vibration_z", vibration.vibration_z);
        JsonUpdateNumber(root, "clipping_0", vibration.clipping_0);
        JsonUpdateNumber(root, "clipping_1", vibration.clipping_1);
        JsonUpdateNumber(root, "clipping_2", vibration.clipping_2);
        break;
    }

    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
        mavlink_servo_output_raw_t servo_output;
        mavlink_msg_servo_output_raw_decode(msg, &servo_output);
        printf("Servo Output Raw - Servo1: %d, Servo2: %d, Servo3: %d, Servo4: %d\n",
               servo_output.servo1_raw, servo_output.servo2_raw, servo_output.servo3_raw, servo_output.servo4_raw);
        JsonUpdateNumber(root, "servo1_raw", servo_output.servo1_raw);
        JsonUpdateNumber(root, "servo2_raw", servo_output.servo2_raw);
        JsonUpdateNumber(root, "servo3_raw", servo_output.servo3_raw);
        JsonUpdateNumber(root, "servo4_raw", servo_output.servo4_raw);
        break;
    }

    case MAVLINK_MSG_ID_RC_CHANNELS: {
        mavlink_rc_channels_t rc_channels;
        mavlink_msg_rc_channels_decode(msg, &rc_channels);
        printf("RC Channels - Chan1: %d, Chan2: %d, Chan3: %d, Chan4: %d\n",
               rc_channels.chan1_raw, rc_channels.chan2_raw, rc_channels.chan3_raw, rc_channels.chan4_raw);
        JsonUpdateNumber(root, "chan1_raw", rc_channels.chan1_raw);
        JsonUpdateNumber(root, "chan2_raw", rc_channels.chan2_raw);
        JsonUpdateNumber(root, "chan3_raw", rc_channels.chan3_raw);
        JsonUpdateNumber(root, "chan4_raw", rc_channels.chan4_raw);
        break;
    }

    case MAVLINK_MSG_ID_TERRAIN_REPORT: {
        mavlink_terrain_report_t terrain_report;
        mavlink_msg_terrain_report_decode(msg, &terrain_report);
        printf("Terrain Report - Lat: %ld, Lon: %ld, Terrain Height: %.2f\n",
               terrain_report.lat, terrain_report.lon, terrain_report.terrain_height);
        JsonUpdateNumber(root, "terrain_lat", terrain_report.lat);
        JsonUpdateNumber(root, "terrain_lon", terrain_report.lon);
        JsonUpdateNumber(root, "terrain_height", terrain_report.terrain_height);
        break;
    }

    default:
        printf("Unknown message ID: %d\n", msg->msgid);
        break;
    }
}
void uart_recevier(void)
{
    cJSON *root;
    uart_event_t event;
    uint8_t *data = (uint8_t *)malloc(1024);
    if (data == NULL) {
        ESP_LOGE(TAG, "Memory allocation failed!");
        vTaskDelete(NULL);
        return;
    }
    BaseType_t durum;
    BaseType_t result1 = xQueueReceive(cJsonHandle,root,portMAX_DELAY);
    if (result1 != pdTRUE) {
    ESP_LOGE(TAG, "Veri kuyruğundan alınamadı!");
    while (result1 != pdTRUE)	
    {
        result1 = xQueueReceive(cJsonHandle,root,portMAX_DELAY);
        ESP_LOGE(TAG, "Veri kuyruğundan alınmaya calisiyor!");
    }
        }
    if (result1 == pdTRUE)
    {
     ESP_LOGE(TAG, "Veri kuyruğundan alındi!");   
    }
    else
    {
        ESP_LOGE(TAG, "Veri kuyruğundan alınamadı!");
        free(data);
        vTaskDelete(NULL);

    }
    while (1)
    {
       durum = xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY);
       if (durum == pdTRUE)
       {
        switch (event.type)
   {
   case UART_DATA:
        int length = uart_read_bytes(UART_NUM_2, data, 1024, pdMS_TO_TICKS(10));
        for (int i = 0; i < length; i++)
        {
            printf("Received byte: 0x%02X\n", data[i]);
            if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status))
            {
                handleMavlinkMessage(root,&msg);
                
            }
            
            /*if (data[0] == 0xFD)
            {
                if (uart_read_bytes(UART_NUM_2,data+1,9,portMAX_DELAY) == 5)
                {
                    if (uart_read_bytes(UART_NUM_2, data+10, data[1]+ 2, portMAX_DELAY) == data[1]+ 2)
                    {
                        uart_mavlink(data);
                        break;
                    }
                    
                }
                
            }*/
            
        }
        
    break;
   
   default:
    break;
       }
}
    }
    free(data);
    vTaskDelete(NULL); 
      // Bellek serbest bırakılıyor
}
void uart_init(void){
    const uart_port_t uart_num = UART_NUM_2;
    uart_config_t uart_config = {
        .baud_rate = 57600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
         .source_clk = UART_SCLK_APB,
        
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num,&uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 5, 4, 17, 18));
    
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2,UART_BUFFER_SIZE,UART_BUFFER_SIZE,10,&uart_queue,0));
    ESP_ERROR_CHECK(uart_set_mode(uart_num,UART_MODE_UART));



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
    cJsonHandle = xQueueCreate(2, sizeof(cJSON *));
    if (cJsonHandle == NULL) {
    printf("Kuyruk oluşturulamadı!\n");
}
    wifi();
    uart_init();
    print_esp32_ap_ip();
    
    xTaskCreate(uart_recevier, "UART Receiver Task", 4096, NULL, 10, NULL);
}