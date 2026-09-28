#ifndef __HIGH_SPEED_LOG_H__
#define __HIGH_SPEED_LOG_H__

#include "main.h"

#define HIGH_SPEED_LOG_MAX_EVENTS 4U

typedef struct
{
  uint16_t dt_us;
  int16_t roll_cdeg;
  int16_t pitch_cdeg;
  int16_t roll_target_rate_dps10;
  int16_t pitch_target_rate_dps10;
  int16_t gyro_x_dps10;
  int16_t gyro_y_dps10;
  int16_t roll_output_pwm10;
  int16_t pitch_output_pwm10;
} high_speed_log_sample_t;

typedef struct
{
  uint8_t event_id;
  uint8_t trigger_axis;
  uint8_t trigger_reason;
  uint32_t trigger_tick_ms;
  uint32_t trigger_control_sequence;
  uint16_t trigger_sample_index;
  uint16_t sample_count;
  int16_t peak_gyro_x_dps10;
  int16_t peak_gyro_y_dps10;
  int16_t peak_roll_error_dps10;
  int16_t peak_pitch_error_dps10;
} high_speed_log_event_info_t;

void high_speed_log_start(void);
void high_speed_log_stop(void);
void high_speed_log_record(uint32_t throttle,
                           float roll_deg,
                           float pitch_deg,
                           float roll_target_rate_dps,
                           float pitch_target_rate_dps,
                           float gyro_x_dps,
                           float gyro_y_dps,
                           float roll_rate_error_dps,
                           float pitch_rate_error_dps,
                           float roll_output,
                           float pitch_output);
void high_speed_log_request_dump(void);
uint8_t high_speed_log_dump_requested(void);
void high_speed_log_finish_dump(void);
uint8_t high_speed_log_get_event_count(void);
uint8_t high_speed_log_get_event_info(uint8_t event_index,
                                      high_speed_log_event_info_t *info);
uint8_t high_speed_log_get_sample(uint8_t event_index,
                                  uint16_t sample_index,
                                  high_speed_log_sample_t *sample);
uint8_t high_speed_log_get_pending_notification(high_speed_log_event_info_t *info);
void high_speed_log_ack_notification(uint8_t event_id);

#endif
