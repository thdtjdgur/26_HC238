#include "flight_control.h"

#include "high_speed_log.h"
#include "motor.h"
#include "sensor.h"

#include <math.h>

#define FLIGHT_THROTTLE_MIN_COMPARE       1000U
#define FLIGHT_THROTTLE_ARM_MAX_COMPARE   1020U
#define FLIGHT_THROTTLE_MAX_COMPARE       1800U
#define FLIGHT_MAX_ANGLE_COMMAND_DEG         25.0f
#define FLIGHT_MAX_YAW_RATE_DPS             120.0f
#define FLIGHT_MAX_ARM_TILT_DEG               15.0f
#define FLIGHT_MAX_ARM_COMMAND_ANGLE_DEG        2.0f
#define FLIGHT_MAX_ARM_YAW_RATE_DPS             5.0f
#define FLIGHT_MAX_OPERATION_TILT_DEG         60.0f
#define FLIGHT_MIN_ARM_VOLTAGE                13.4f
#define FLIGHT_LOW_BATTERY_VOLTAGE            13.2f
#define FLIGHT_LOW_BATTERY_HOLD_MS          2000U
#define FLIGHT_COMMAND_TIMEOUT_MS             500U
#define FLIGHT_IMU_TIMEOUT_MS                  20U

typedef struct
{
  uint16_t throttle_compare;
  float roll_angle_deg;
  float pitch_angle_deg;
  float yaw_rate_dps;
  uint32_t received_tick_ms;
  uint8_t valid;
} flight_command_t;

static flight_state_t flight_state = FLIGHT_STATE_DISARMED;
static flight_failsafe_reason_t failsafe_reason = FLIGHT_FAILSAFE_NONE;
static volatile flight_command_t command;
static uint32_t low_battery_start_tick_ms = 0U;

static flight_command_t FlightCommandSnapshot(void)
{
  flight_command_t snapshot;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  snapshot.throttle_compare = command.throttle_compare;
  snapshot.roll_angle_deg = command.roll_angle_deg;
  snapshot.pitch_angle_deg = command.pitch_angle_deg;
  snapshot.yaw_rate_dps = command.yaw_rate_dps;
  snapshot.received_tick_ms = command.received_tick_ms;
  snapshot.valid = command.valid;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return snapshot;
}

static float FlightClamp(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

static uint8_t FlightAttitudeFinite(void)
{
  return (isfinite(sensor_roll_deg) &&
          isfinite(sensor_pitch_deg) &&
          isfinite(sensor_yaw_deg)) ? 1U : 0U;
}

static void FlightEnterFailsafe(flight_failsafe_reason_t reason)
{
  failsafe_reason = reason;
  flight_state = FLIGHT_STATE_FAILSAFE;
  high_speed_log_stop();
  motor_set_armed(0U);
}

void flight_control_init(void)
{
  command.throttle_compare = FLIGHT_THROTTLE_MIN_COMPARE;
  command.roll_angle_deg = 0.0f;
  command.pitch_angle_deg = 0.0f;
  command.yaw_rate_dps = 0.0f;
  command.received_tick_ms = 0U;
  command.valid = 0U;
  low_battery_start_tick_ms = 0U;
  failsafe_reason = FLIGHT_FAILSAFE_NONE;
  flight_state = FLIGHT_STATE_DISARMED;
  motor_set_armed(0U);
}

bool flight_control_set_command(uint16_t throttle_compare,
                                float roll_angle_deg,
                                float pitch_angle_deg,
                                float yaw_rate_dps)
{
  uint32_t primask;

  if ((!isfinite(roll_angle_deg)) ||
      (!isfinite(pitch_angle_deg)) ||
      (!isfinite(yaw_rate_dps)))
  {
    return false;
  }

  if (throttle_compare < FLIGHT_THROTTLE_MIN_COMPARE)
  {
    throttle_compare = FLIGHT_THROTTLE_MIN_COMPARE;
  }
  else if (throttle_compare > FLIGHT_THROTTLE_MAX_COMPARE)
  {
    throttle_compare = FLIGHT_THROTTLE_MAX_COMPARE;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  command.throttle_compare = throttle_compare;
  command.roll_angle_deg = FlightClamp(roll_angle_deg,
                                       -FLIGHT_MAX_ANGLE_COMMAND_DEG,
                                        FLIGHT_MAX_ANGLE_COMMAND_DEG);
  command.pitch_angle_deg = FlightClamp(pitch_angle_deg,
                                        -FLIGHT_MAX_ANGLE_COMMAND_DEG,
                                         FLIGHT_MAX_ANGLE_COMMAND_DEG);
  command.yaw_rate_dps = FlightClamp(yaw_rate_dps,
                                     -FLIGHT_MAX_YAW_RATE_DPS,
                                      FLIGHT_MAX_YAW_RATE_DPS);
  command.received_tick_ms = HAL_GetTick();
  command.valid = 1U;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return true;
}

bool flight_control_request_arm(float battery_voltage, uint8_t battery_valid)
{
  uint32_t now = HAL_GetTick();
  flight_command_t current_command = FlightCommandSnapshot();

  if (flight_state != FLIGHT_STATE_DISARMED)
  {
    return false;
  }
  if ((current_command.valid == 0U) ||
      ((uint32_t)(now - current_command.received_tick_ms) > FLIGHT_COMMAND_TIMEOUT_MS) ||
      (current_command.throttle_compare > FLIGHT_THROTTLE_ARM_MAX_COMPARE) ||
      (fabsf(current_command.roll_angle_deg) > FLIGHT_MAX_ARM_COMMAND_ANGLE_DEG) ||
      (fabsf(current_command.pitch_angle_deg) > FLIGHT_MAX_ARM_COMMAND_ANGLE_DEG) ||
      (fabsf(current_command.yaw_rate_dps) > FLIGHT_MAX_ARM_YAW_RATE_DPS))
  {
    return false;
  }
  if ((sensor_is_healthy(FLIGHT_IMU_TIMEOUT_MS) == 0U) ||
      (FlightAttitudeFinite() == 0U) ||
      (fabsf(sensor_roll_deg) > FLIGHT_MAX_ARM_TILT_DEG) ||
      (fabsf(sensor_pitch_deg) > FLIGHT_MAX_ARM_TILT_DEG))
  {
    return false;
  }
  if ((battery_valid == 0U) || (!isfinite(battery_voltage)) ||
      (battery_voltage < FLIGHT_MIN_ARM_VOLTAGE))
  {
    return false;
  }

  failsafe_reason = FLIGHT_FAILSAFE_NONE;
  low_battery_start_tick_ms = 0U;
  motor_set_armed(1U);
  high_speed_log_start();
  flight_state = FLIGHT_STATE_ARMED;
  return true;
}

void flight_control_disarm(flight_failsafe_reason_t reason)
{
  motor_set_armed(0U);
  high_speed_log_stop();
  command.throttle_compare = FLIGHT_THROTTLE_MIN_COMPARE;
  command.roll_angle_deg = 0.0f;
  command.pitch_angle_deg = 0.0f;
  command.yaw_rate_dps = 0.0f;
  command.valid = 0U;
  low_battery_start_tick_ms = 0U;
  failsafe_reason = reason;
  flight_state = (reason == FLIGHT_FAILSAFE_NONE || reason == FLIGHT_FAILSAFE_USER) ?
                 FLIGHT_STATE_DISARMED : FLIGHT_STATE_FAILSAFE;
}

void flight_control_process(float battery_voltage, uint8_t battery_valid)
{
  uint32_t now;
  flight_command_t current_command;

  if (flight_state != FLIGHT_STATE_ARMED)
  {
    motor_set_armed(0U);
    return;
  }

  now = HAL_GetTick();
  current_command = FlightCommandSnapshot();
  if ((current_command.valid == 0U) ||
      ((uint32_t)(now - current_command.received_tick_ms) > FLIGHT_COMMAND_TIMEOUT_MS))
  {
    FlightEnterFailsafe(FLIGHT_FAILSAFE_COMMAND_TIMEOUT);
    return;
  }
  if ((sensor_is_healthy(FLIGHT_IMU_TIMEOUT_MS) == 0U) ||
      (FlightAttitudeFinite() == 0U))
  {
    FlightEnterFailsafe(FLIGHT_FAILSAFE_IMU_TIMEOUT);
    return;
  }
  if ((fabsf(sensor_roll_deg) > FLIGHT_MAX_OPERATION_TILT_DEG) ||
      (fabsf(sensor_pitch_deg) > FLIGHT_MAX_OPERATION_TILT_DEG))
  {
    FlightEnterFailsafe(FLIGHT_FAILSAFE_EXCESSIVE_TILT);
    return;
  }

  if ((battery_valid != 0U) && isfinite(battery_voltage) &&
      (battery_voltage < FLIGHT_LOW_BATTERY_VOLTAGE))
  {
    if (low_battery_start_tick_ms == 0U)
    {
      low_battery_start_tick_ms = now;
    }
    else if ((uint32_t)(now - low_battery_start_tick_ms) >= FLIGHT_LOW_BATTERY_HOLD_MS)
    {
      FlightEnterFailsafe(FLIGHT_FAILSAFE_LOW_BATTERY);
      return;
    }
  }
  else
  {
    low_battery_start_tick_ms = 0U;
  }

  motor_set_throttle(current_command.throttle_compare);
  motor_set_angle_targets(current_command.roll_angle_deg, current_command.pitch_angle_deg);
  motor_set_rate_targets(0.0f, 0.0f, current_command.yaw_rate_dps);
  motor_rate_pid_update();
}

flight_state_t flight_control_get_state(void)
{
  return flight_state;
}

flight_failsafe_reason_t flight_control_get_failsafe_reason(void)
{
  return failsafe_reason;
}

const char *flight_control_state_text(void)
{
  switch (flight_state)
  {
    case FLIGHT_STATE_ARMED: return "ARMED";
    case FLIGHT_STATE_FAILSAFE: return "FAILSAFE";
    case FLIGHT_STATE_DISARMED:
    default: return "DISARMED";
  }
}

const char *flight_control_failsafe_text(void)
{
  switch (failsafe_reason)
  {
    case FLIGHT_FAILSAFE_USER: return "USER";
    case FLIGHT_FAILSAFE_COMMAND_TIMEOUT: return "COMMAND_TIMEOUT";
    case FLIGHT_FAILSAFE_IMU_TIMEOUT: return "IMU_TIMEOUT";
    case FLIGHT_FAILSAFE_EXCESSIVE_TILT: return "EXCESSIVE_TILT";
    case FLIGHT_FAILSAFE_LOW_BATTERY: return "LOW_BATTERY";
    case FLIGHT_FAILSAFE_EMERGENCY_INPUT: return "EMERGENCY_INPUT";
    case FLIGHT_FAILSAFE_NONE:
    default: return "NONE";
  }
}
