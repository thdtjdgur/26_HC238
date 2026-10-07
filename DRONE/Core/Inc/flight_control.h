#ifndef __FLIGHT_CONTROL_H__
#define __FLIGHT_CONTROL_H__

#include "main.h"

#include <stdbool.h>

typedef enum
{
  FLIGHT_STATE_DISARMED = 0,
  FLIGHT_STATE_ARMED,
  FLIGHT_STATE_FAILSAFE
} flight_state_t;

typedef enum
{
  FLIGHT_MODE_ANGLE = 0,
  FLIGHT_MODE_GPS
} flight_mode_t;

typedef enum
{
  FLIGHT_FAILSAFE_NONE = 0,
  FLIGHT_FAILSAFE_USER,
  FLIGHT_FAILSAFE_COMMAND_TIMEOUT,
  FLIGHT_FAILSAFE_IMU_TIMEOUT,
  FLIGHT_FAILSAFE_EXCESSIVE_TILT,
  FLIGHT_FAILSAFE_LOW_BATTERY,
  FLIGHT_FAILSAFE_EMERGENCY_INPUT,
  FLIGHT_FAILSAFE_GPS_TIMEOUT
} flight_failsafe_reason_t;

void flight_control_init(void);
bool flight_control_set_command(uint16_t throttle_compare,
                                float roll_angle_deg,
                                float pitch_angle_deg,
                                float yaw_rate_dps);
bool flight_control_set_mode(flight_mode_t mode);
bool flight_control_set_gps_target(int32_t latitude_deg_1e7,
                                   int32_t longitude_deg_1e7,
                                   uint16_t throttle_compare);
bool flight_control_refresh_command(void);
bool flight_control_request_arm(float battery_voltage, uint8_t battery_valid);
void flight_control_disarm(flight_failsafe_reason_t reason);
void flight_control_process(float battery_voltage, uint8_t battery_valid);
flight_state_t flight_control_get_state(void);
flight_mode_t flight_control_get_mode(void);
flight_failsafe_reason_t flight_control_get_failsafe_reason(void);
const char *flight_control_state_text(void);
const char *flight_control_mode_text(void);
const char *flight_control_failsafe_text(void);

#endif
