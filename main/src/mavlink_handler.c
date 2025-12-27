#include "mavlink_handler.h"
#include "data_manager.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "common/mavlink.h"
#pragma GCC diagnostic pop
#include "esp_log.h"

static const char *TAG = "MAVLINK_HANDLER";

static mavlink_message_t msg;
static mavlink_status_t status;

void mavlink_handler_init(void) {
    // Initialization if any needed (e.g. stats counters)
}

static void handle_message(const mavlink_message_t *msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(msg, &heartbeat);
            data_manager_set_heartbeat(heartbeat.custom_mode, heartbeat.type, heartbeat.autopilot, 
                                     heartbeat.base_mode, heartbeat.system_status, heartbeat.mavlink_version);
            break;
        }

        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(msg, &sys_status);
            data_manager_set_sys_status(sys_status.voltage_battery, sys_status.battery_remaining);
            break;
        }

        case MAVLINK_MSG_ID_GPS_RAW_INT: {
            mavlink_gps_raw_int_t gps_raw;
            mavlink_msg_gps_raw_int_decode(msg, &gps_raw);
            data_manager_set_gps_raw(gps_raw.lat, gps_raw.lon, gps_raw.alt, gps_raw.fix_type);
            break;
        }

        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(msg, &attitude);
            data_manager_set_attitude(attitude.roll, attitude.pitch, attitude.yaw);
            break;
        }

        case MAVLINK_MSG_ID_GPS_STATUS: {
            mavlink_gps_status_t gps_status;
            mavlink_msg_gps_status_decode(msg, &gps_status);
            data_manager_set_gps_status(gps_status.satellites_visible);
            break;
        }

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t global_pos;
            mavlink_msg_global_position_int_decode(msg, &global_pos);
            data_manager_set_global_position(global_pos.lat, global_pos.lon, global_pos.alt);
            break;
        }

        case MAVLINK_MSG_ID_POWER_STATUS: {
            mavlink_power_status_t power_status;
            mavlink_msg_power_status_decode(msg, &power_status);
            data_manager_set_power_status(power_status.Vcc, power_status.Vservo, power_status.flags);
            break;
        }

        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: {
            mavlink_nav_controller_output_t nav_output;
            mavlink_msg_nav_controller_output_decode(msg, &nav_output);
            data_manager_set_nav_controller_output(nav_output.target_bearing, nav_output.xtrack_error);
            break;
        }

        case MAVLINK_MSG_ID_MISSION_CURRENT: {
            mavlink_mission_current_t mission_current;
            mavlink_msg_mission_current_decode(msg, &mission_current);
            data_manager_set_mission_current(mission_current.seq);
            break;
        }

        case MAVLINK_MSG_ID_VFR_HUD: {
            mavlink_vfr_hud_t vfr_hud;
            mavlink_msg_vfr_hud_decode(msg, &vfr_hud);
            data_manager_set_vfr_hud(vfr_hud.airspeed, vfr_hud.alt);
            break;
        }

        case MAVLINK_MSG_ID_RAW_IMU: {
            mavlink_raw_imu_t raw_imu;
            mavlink_msg_raw_imu_decode(msg, &raw_imu);
            data_manager_set_raw_imu(raw_imu.xacc, raw_imu.yacc, raw_imu.zacc, 
                                     raw_imu.xgyro, raw_imu.ygyro, raw_imu.zgyro);
            break;
        }

        case MAVLINK_MSG_ID_SCALED_PRESSURE: {
            mavlink_scaled_pressure_t scaled_pressure;
            mavlink_msg_scaled_pressure_decode(msg, &scaled_pressure);
            data_manager_set_scaled_pressure(scaled_pressure.press_abs, scaled_pressure.temperature);
            break;
        }

        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t command;
            mavlink_msg_command_long_decode(msg, &command);
            data_manager_set_command_long(command.command, command.param1, command.param2);
            break;
        }

        case MAVLINK_MSG_ID_STATUSTEXT: {
            mavlink_statustext_t status_text;
            mavlink_msg_statustext_decode(msg, &status_text);
            data_manager_set_statustext(status_text.severity, status_text.text);
            break;
        }

        case MAVLINK_MSG_ID_VIBRATION: {
            mavlink_vibration_t vibration;
            mavlink_msg_vibration_decode(msg, &vibration);
            data_manager_set_vibration(vibration.vibration_x, vibration.vibration_y, vibration.vibration_z,
                                       vibration.clipping_0, vibration.clipping_1, vibration.clipping_2);
            break;
        }

        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
            mavlink_servo_output_raw_t servo_output;
            mavlink_msg_servo_output_raw_decode(msg, &servo_output);
            data_manager_set_servo_output_raw(servo_output.servo1_raw, servo_output.servo2_raw, 
                                            servo_output.servo3_raw, servo_output.servo4_raw);
            break;
        }

        case MAVLINK_MSG_ID_RC_CHANNELS: {
            mavlink_rc_channels_t rc_channels;
            mavlink_msg_rc_channels_decode(msg, &rc_channels);
            data_manager_set_rc_channels(rc_channels.chan1_raw, rc_channels.chan2_raw, 
                                       rc_channels.chan3_raw, rc_channels.chan4_raw);
            break;
        }

        case MAVLINK_MSG_ID_TERRAIN_REPORT: {
            mavlink_terrain_report_t terrain_report;
            mavlink_msg_terrain_report_decode(msg, &terrain_report);
            data_manager_set_terrain_report(terrain_report.lat, terrain_report.lon, terrain_report.terrain_height);
            break;
        }

        default:
            ESP_LOGD(TAG, "Unhandled message ID: %d", msg->msgid);
            break;
    }
}

void mavlink_handler_parse_byte(uint8_t byte) {
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
        handle_message(&msg);
    }
}
