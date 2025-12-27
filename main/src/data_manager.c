#include "data_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "DATA_MANAGER";

static MavlinkData shared_data = {0};
static SemaphoreHandle_t data_mutex = NULL;

void data_manager_init(void) {
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create data mutex");
    }
}

void data_manager_get_data(MavlinkData *out_data) {
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        *out_data = shared_data;
        xSemaphoreGive(data_mutex);
    }
}

// Helper macro to wrap mutex logic
#define UPDATE_DATA(block) \
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) { \
        block \
        xSemaphoreGive(data_mutex); \
    }

void data_manager_set_heartbeat(int custom_mode, int type, int autopilot, int base_mode, int system_status, int mavlink_version) {
    UPDATE_DATA({
        shared_data.custom_mode = custom_mode;
        shared_data.type = type;
        shared_data.autopilot = autopilot;
        shared_data.base_mode = base_mode;
        shared_data.system_status = system_status;
        shared_data.mavlink_version = mavlink_version;
    })
}

void data_manager_set_sys_status(int battery_voltage, int battery_remaining) {
    UPDATE_DATA({
        shared_data.battery_voltage = battery_voltage;
        shared_data.battery_remaining = battery_remaining;
    })
}

void data_manager_set_gps_raw(long lat, long lon, long alt, int fix_type) {
    UPDATE_DATA({
        shared_data.lat = lat;
        shared_data.lon = lon;
        shared_data.alt = alt;
        shared_data.fix_type = fix_type;
    })
}

void data_manager_set_attitude(float roll, float pitch, float yaw) {
    UPDATE_DATA({
        shared_data.roll = roll;
        shared_data.pitch = pitch;
        shared_data.yaw = yaw;
    })
}

void data_manager_set_gps_status(int satellites_visible) {
    UPDATE_DATA({
        shared_data.satellites_visible = satellites_visible;
    })
}

void data_manager_set_global_position(long lat, long lon, long alt) {
    UPDATE_DATA({
        shared_data.global_lat = lat;
        shared_data.global_lon = lon;
        shared_data.global_alt = alt;
    })
}

void data_manager_set_power_status(int vcc, int vservo, int flags) {
    UPDATE_DATA({
        shared_data.vcc = vcc;
        shared_data.vservo = vservo;
        shared_data.flags = flags;
    })
}

void data_manager_set_nav_controller_output(int target_bearing, float xtrack_error) {
    UPDATE_DATA({
        shared_data.target_bearing = target_bearing;
        shared_data.xtrack_error = xtrack_error;
    })
}

void data_manager_set_mission_current(int seq) {
    UPDATE_DATA({
        shared_data.mission_seq = seq;
    })
}

void data_manager_set_vfr_hud(float airspeed, float alt) {
    UPDATE_DATA({
        shared_data.airspeed = airspeed;
        shared_data.altitude = alt;
    })
}

void data_manager_set_raw_imu(int xacc, int yacc, int zacc, int xgyro, int ygyro, int zgyro) {
    UPDATE_DATA({
        shared_data.xacc = xacc;
        shared_data.yacc = yacc;
        shared_data.zacc = zacc;
        shared_data.xgyro = xgyro;
        shared_data.ygyro = ygyro;
        shared_data.zgyro = zgyro;
    })
}

void data_manager_set_scaled_pressure(float pressure, int temperature) {
    UPDATE_DATA({
        shared_data.pressure = pressure;
        shared_data.temperature = temperature;
    })
}

void data_manager_set_command_long(int command, float param1, float param2) {
    UPDATE_DATA({
        shared_data.command = command;
        shared_data.param1 = param1;
        shared_data.param2 = param2;
    })
}

void data_manager_set_statustext(int severity, const char *text) {
    UPDATE_DATA({
        shared_data.severity = severity;
        strncpy(shared_data.message, text, sizeof(shared_data.message) - 1);
        shared_data.message[sizeof(shared_data.message) - 1] = '\0';
    })
}

void data_manager_set_vibration(float vib_x, float vib_y, float vib_z, long clip0, long clip1, long clip2) {
    UPDATE_DATA({
        shared_data.vibration_x = vib_x;
        shared_data.vibration_y = vib_y;
        shared_data.vibration_z = vib_z;
        shared_data.clipping_0 = clip0;
        shared_data.clipping_1 = clip1;
        shared_data.clipping_2 = clip2;
    })
}

void data_manager_set_servo_output_raw(int s1, int s2, int s3, int s4) {
    UPDATE_DATA({
        shared_data.servo1_raw = s1;
        shared_data.servo2_raw = s2;
        shared_data.servo3_raw = s3;
        shared_data.servo4_raw = s4;
    })
}

void data_manager_set_rc_channels(int c1, int c2, int c3, int c4) {
    UPDATE_DATA({
        shared_data.chan1_raw = c1;
        shared_data.chan2_raw = c2;
        shared_data.chan3_raw = c3;
        shared_data.chan4_raw = c4;
    })
}

void data_manager_set_terrain_report(long lat, long lon, float height) {
    UPDATE_DATA({
        shared_data.terrain_lat = lat;
        shared_data.terrain_lon = lon;
        shared_data.terrain_height = height;
    })
}


// Performance note: Creating cJSON object every time is safer but slower. 
// Given the requirements, we will implement the robust cJSON method first.
char* data_manager_get_json_string(void) {
    MavlinkData local_data;
    data_manager_get_data(&local_data);

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return NULL;

    cJSON_AddNumberToObject(root, "id", local_data.id);
    cJSON_AddNumberToObject(root, "temperature", local_data.temperature);
    cJSON_AddNumberToObject(root, "speed", local_data.speed);
    cJSON_AddStringToObject(root, "message", local_data.message);
    cJSON_AddNumberToObject(root, "custom_mode", local_data.custom_mode);
    cJSON_AddNumberToObject(root, "type", local_data.type);
    cJSON_AddNumberToObject(root, "autopilot", local_data.autopilot);
    cJSON_AddNumberToObject(root, "base_mode", local_data.base_mode);
    cJSON_AddNumberToObject(root, "system_status", local_data.system_status);
    cJSON_AddNumberToObject(root, "mavlink_version", local_data.mavlink_version);
    cJSON_AddNumberToObject(root, "battery_voltage", local_data.battery_voltage);
    cJSON_AddNumberToObject(root, "battery_remaining", local_data.battery_remaining);
    cJSON_AddNumberToObject(root, "lat", local_data.lat);
    cJSON_AddNumberToObject(root, "lon", local_data.lon);
    cJSON_AddNumberToObject(root, "alt", local_data.alt);
    cJSON_AddNumberToObject(root, "fix_type", local_data.fix_type);
    cJSON_AddNumberToObject(root, "roll", local_data.roll);
    cJSON_AddNumberToObject(root, "pitch", local_data.pitch);
    cJSON_AddNumberToObject(root, "yaw", local_data.yaw);
    cJSON_AddNumberToObject(root, "satellites_visible", local_data.satellites_visible);
    cJSON_AddNumberToObject(root, "global_lat", local_data.global_lat);
    cJSON_AddNumberToObject(root, "global_lon", local_data.global_lon);
    cJSON_AddNumberToObject(root, "global_alt", local_data.global_alt);
    cJSON_AddNumberToObject(root, "vcc", local_data.vcc);
    cJSON_AddNumberToObject(root, "vservo", local_data.vservo);
    cJSON_AddNumberToObject(root, "flags", local_data.flags);
    cJSON_AddNumberToObject(root, "target_bearing", local_data.target_bearing);
    cJSON_AddNumberToObject(root, "xtrack_error", local_data.xtrack_error);
    cJSON_AddNumberToObject(root, "mission_seq", local_data.mission_seq);
    cJSON_AddNumberToObject(root, "airspeed", local_data.airspeed);
    cJSON_AddNumberToObject(root, "altitude", local_data.altitude);
    cJSON_AddNumberToObject(root, "xacc", local_data.xacc);
    cJSON_AddNumberToObject(root, "yacc", local_data.yacc);
    cJSON_AddNumberToObject(root, "zacc", local_data.zacc);
    cJSON_AddNumberToObject(root, "xgyro", local_data.xgyro);
    cJSON_AddNumberToObject(root, "ygyro", local_data.ygyro);
    cJSON_AddNumberToObject(root, "zgyro", local_data.zgyro);
    cJSON_AddNumberToObject(root, "pressure", local_data.pressure);
    cJSON_AddNumberToObject(root, "command", local_data.command);
    cJSON_AddNumberToObject(root, "param1", local_data.param1);
    cJSON_AddNumberToObject(root, "param2", local_data.param2);
    cJSON_AddNumberToObject(root, "severity", local_data.severity);
    cJSON_AddStringToObject(root, "status_message", local_data.status_message);
    cJSON_AddNumberToObject(root, "vibration_x", local_data.vibration_x);
    cJSON_AddNumberToObject(root, "vibration_y", local_data.vibration_y);
    cJSON_AddNumberToObject(root, "vibration_z", local_data.vibration_z);
    cJSON_AddNumberToObject(root, "clipping_0", local_data.clipping_0);
    cJSON_AddNumberToObject(root, "clipping_1", local_data.clipping_1);
    cJSON_AddNumberToObject(root, "clipping_2", local_data.clipping_2);
    cJSON_AddNumberToObject(root, "servo1_raw", local_data.servo1_raw);
    cJSON_AddNumberToObject(root, "servo2_raw", local_data.servo2_raw);
    cJSON_AddNumberToObject(root, "servo3_raw", local_data.servo3_raw);
    cJSON_AddNumberToObject(root, "servo4_raw", local_data.servo4_raw);
    cJSON_AddNumberToObject(root, "chan1_raw", local_data.chan1_raw);
    cJSON_AddNumberToObject(root, "chan2_raw", local_data.chan2_raw);
    cJSON_AddNumberToObject(root, "chan3_raw", local_data.chan3_raw);
    cJSON_AddNumberToObject(root, "chan4_raw", local_data.chan4_raw);
    cJSON_AddNumberToObject(root, "terrain_lat", local_data.terrain_lat);
    cJSON_AddNumberToObject(root, "terrain_lon", local_data.terrain_lon);
    cJSON_AddNumberToObject(root, "terrain_height", local_data.terrain_height);

    char *json_string = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json_string;
}
