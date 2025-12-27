#ifndef DATA_MANAGER_H
#define DATA_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "cJSON.h"

// Define the Mavlink Data Structure
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

/**
 * @brief Initialize the data manager (mutexes, etc.)
 */
void data_manager_init(void);

/**
 * @brief Get a copy of the current Mavlink data in a thread-safe way.
 * 
 * @param out_data Pointer to the struct to copy data into.
 */
void data_manager_get_data(MavlinkData *out_data);

/**
 * @brief Get a specific field or update a specific field. 
 * Note: For this refactor, we will likely expose setters for groups of data 
 * or pointers to the shared struct with a lock, but for simplicity/performance
 * let's expose setters that the Mavlink handler will use.
 */

// Thread-safe setters for specific groups (matching Mavlink messages)
void data_manager_set_heartbeat(int custom_mode, int type, int autopilot, int base_mode, int system_status, int mavlink_version);
void data_manager_set_sys_status(int battery_voltage, int battery_remaining);
void data_manager_set_gps_raw(long lat, long lon, long alt, int fix_type);
void data_manager_set_attitude(float roll, float pitch, float yaw);
void data_manager_set_gps_status(int satellites_visible);
void data_manager_set_global_position(long lat, long lon, long alt);
void data_manager_set_power_status(int vcc, int vservo, int flags);
void data_manager_set_nav_controller_output(int target_bearing, float xtrack_error);
void data_manager_set_mission_current(int seq);
void data_manager_set_vfr_hud(float airspeed, float alt);
void data_manager_set_raw_imu(int xacc, int yacc, int zacc, int xgyro, int ygyro, int zgyro);
void data_manager_set_scaled_pressure(float pressure, int temperature);
void data_manager_set_command_long(int command, float param1, float param2);
void data_manager_set_statustext(int severity, const char *text);
void data_manager_set_vibration(float vib_x, float vib_y, float vib_z, long clip0, long clip1, long clip2);
void data_manager_set_servo_output_raw(int s1, int s2, int s3, int s4);
void data_manager_set_rc_channels(int c1, int c2, int c3, int c4);
void data_manager_set_terrain_report(long lat, long lon, float height);

/**
 * @brief Generate a JSON string representing the current state.
 * Caller is responsible for freeing the returned string.
 * 
 * @return char* JSON string
 */
char* data_manager_get_json_string(void);

#endif // DATA_MANAGER_H
