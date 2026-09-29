#include "high_speed_log.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define HIGH_SPEED_LOG_ARM_THROTTLE          1425U
#define HIGH_SPEED_LOG_PRE_SAMPLES           1536U
#define HIGH_SPEED_LOG_POST_SAMPLES          1536U
#define HIGH_SPEED_LOG_EVENT_SAMPLES         (HIGH_SPEED_LOG_PRE_SAMPLES + HIGH_SPEED_LOG_POST_SAMPLES)
#define HIGH_SPEED_LOG_TRIGGER_CONFIRM       20U
#define HIGH_SPEED_LOG_COOLDOWN_SAMPLES      1000U
#define HIGH_SPEED_LOG_GYRO_TRIGGER_DPS       30.0f
#define HIGH_SPEED_LOG_ERROR_TRIGGER_DPS      25.0f
#define HIGH_SPEED_LOG_REASON_ROLL_GYRO        0x01U
#define HIGH_SPEED_LOG_REASON_PITCH_GYRO       0x02U
#define HIGH_SPEED_LOG_REASON_ROLL_ERROR       0x04U
#define HIGH_SPEED_LOG_REASON_PITCH_ERROR      0x08U

_Static_assert(sizeof(high_speed_log_sample_t) == 18U, "Unexpected high-speed sample size");

static high_speed_log_sample_t event_samples[HIGH_SPEED_LOG_MAX_EVENTS][HIGH_SPEED_LOG_EVENT_SAMPLES]
  __attribute__((aligned(32), section(".dma_buffer")));
static high_speed_log_event_info_t event_info[HIGH_SPEED_LOG_MAX_EVENTS];
static uint16_t event_pre_start[HIGH_SPEED_LOG_MAX_EVENTS];
static uint8_t event_notification_pending[HIGH_SPEED_LOG_MAX_EVENTS];

static uint8_t log_active = 0U;
static uint8_t capture_active = 0U;
static uint8_t event_count = 0U;
static uint8_t current_event_index = 0U;
static uint8_t dump_pending = 0U;
static uint16_t pre_write_index = 0U;
static uint16_t pre_sample_count = 0U;
static uint16_t current_event_sample_count = 0U;
static uint16_t trigger_confirm_count = 0U;
static uint32_t cooldown_sample_count = 0U;
static uint32_t control_sequence = 0U;
static uint32_t previous_record_cycles = 0U;
static uint8_t record_time_initialized = 0U;

static int16_t HighSpeedLogScaleSigned(float value, float scale)
{
  float scaled = value * scale;

  if (scaled > 32767.0f)
  {
    return INT16_MAX;
  }
  if (scaled < -32768.0f)
  {
    return INT16_MIN;
  }
  return (int16_t)scaled;
}

static uint16_t HighSpeedLogMeasureDtUs(void)
{
  uint32_t now_cycles = DWT->CYCCNT;
  uint32_t elapsed_cycles;
  uint32_t elapsed_us = 500U;

  if ((record_time_initialized != 0U) && (SystemCoreClock != 0U))
  {
    elapsed_cycles = now_cycles - previous_record_cycles;
    elapsed_us = (uint32_t)(((uint64_t)elapsed_cycles * 1000000ULL) / SystemCoreClock);
    if (elapsed_us > UINT16_MAX)
    {
      elapsed_us = UINT16_MAX;
    }
  }
  else
  {
    record_time_initialized = 1U;
  }

  previous_record_cycles = now_cycles;
  return (uint16_t)elapsed_us;
}

static void HighSpeedLogUpdateSignedPeak(int16_t value, int16_t *peak)
{
  if ((peak != NULL) && (abs((int)value) > abs((int)*peak)))
  {
    *peak = value;
  }
}

static void HighSpeedLogUpdateEventPeaks(uint8_t event_index,
                                         const high_speed_log_sample_t *sample,
                                         int16_t roll_error_dps10,
                                         int16_t pitch_error_dps10)
{
  high_speed_log_event_info_t *info = &event_info[event_index];

  HighSpeedLogUpdateSignedPeak(sample->gyro_x_dps10, &info->peak_gyro_x_dps10);
  HighSpeedLogUpdateSignedPeak(sample->gyro_y_dps10, &info->peak_gyro_y_dps10);
  HighSpeedLogUpdateSignedPeak(roll_error_dps10, &info->peak_roll_error_dps10);
  HighSpeedLogUpdateSignedPeak(pitch_error_dps10, &info->peak_pitch_error_dps10);
}

static const high_speed_log_sample_t *HighSpeedLogGetStoredSample(uint8_t event_index,
                                                                  uint16_t sample_index)
{
  uint16_t physical_index = sample_index;

  if (sample_index <= event_info[event_index].trigger_sample_index)
  {
    physical_index = (uint16_t)((event_pre_start[event_index] + sample_index) %
                                HIGH_SPEED_LOG_PRE_SAMPLES);
  }

  return &event_samples[event_index][physical_index];
}

static void HighSpeedLogRecalculateEventPeaks(uint8_t event_index)
{
  high_speed_log_event_info_t *info = &event_info[event_index];
  const high_speed_log_sample_t *sample;
  int32_t roll_error_dps10;
  int32_t pitch_error_dps10;
  uint16_t index;

  info->peak_gyro_x_dps10 = 0;
  info->peak_gyro_y_dps10 = 0;
  info->peak_roll_error_dps10 = 0;
  info->peak_pitch_error_dps10 = 0;

  for (index = 0U; index < info->sample_count; index++)
  {
    sample = HighSpeedLogGetStoredSample(event_index, index);
    roll_error_dps10 = (int32_t)sample->roll_target_rate_dps10 - sample->gyro_x_dps10;
    pitch_error_dps10 = (int32_t)sample->pitch_target_rate_dps10 - sample->gyro_y_dps10;
    if (roll_error_dps10 > INT16_MAX)
    {
      roll_error_dps10 = INT16_MAX;
    }
    else if (roll_error_dps10 < INT16_MIN)
    {
      roll_error_dps10 = INT16_MIN;
    }
    if (pitch_error_dps10 > INT16_MAX)
    {
      pitch_error_dps10 = INT16_MAX;
    }
    else if (pitch_error_dps10 < INT16_MIN)
    {
      pitch_error_dps10 = INT16_MIN;
    }
    HighSpeedLogUpdateEventPeaks(event_index,
                                 sample,
                                 (int16_t)roll_error_dps10,
                                 (int16_t)pitch_error_dps10);
  }
}

static void HighSpeedLogFinalizeCurrentEvent(void)
{
  if (capture_active == 0U)
  {
    return;
  }

  event_info[current_event_index].sample_count = current_event_sample_count;
  event_count++;
  capture_active = 0U;
  current_event_sample_count = 0U;
  cooldown_sample_count = HIGH_SPEED_LOG_COOLDOWN_SAMPLES;
}

static void HighSpeedLogStartEvent(uint8_t trigger_axis,
                                   uint8_t trigger_reason,
                                   int16_t roll_error_dps10,
                                   int16_t pitch_error_dps10)
{
  high_speed_log_event_info_t *info;

  if ((event_count >= HIGH_SPEED_LOG_MAX_EVENTS) || (pre_sample_count == 0U))
  {
    return;
  }

  current_event_index = event_count;
  info = &event_info[current_event_index];
  memset(info, 0, sizeof(*info));
  info->event_id = (uint8_t)(current_event_index + 1U);
  info->trigger_reason = trigger_reason;
  info->trigger_tick_ms = HAL_GetTick();
  info->trigger_control_sequence = control_sequence;
  info->trigger_sample_index = (uint16_t)(pre_sample_count - 1U);
  info->trigger_axis = trigger_axis;
  event_pre_start[current_event_index] =
      (uint16_t)((pre_write_index + HIGH_SPEED_LOG_PRE_SAMPLES - pre_sample_count) %
                 HIGH_SPEED_LOG_PRE_SAMPLES);

  HighSpeedLogUpdateSignedPeak(roll_error_dps10, &info->peak_roll_error_dps10);
  HighSpeedLogUpdateSignedPeak(pitch_error_dps10, &info->peak_pitch_error_dps10);
  current_event_sample_count = pre_sample_count;
  capture_active = 1U;
  event_notification_pending[current_event_index] = 1U;
}

void high_speed_log_start(void)
{
  log_active = 1U;
  capture_active = 0U;
  event_count = 0U;
  current_event_index = 0U;
  dump_pending = 0U;
  pre_write_index = 0U;
  pre_sample_count = 0U;
  current_event_sample_count = 0U;
  trigger_confirm_count = 0U;
  cooldown_sample_count = 0U;
  control_sequence = 0U;
  previous_record_cycles = DWT->CYCCNT;
  record_time_initialized = 0U;
  memset(event_info, 0, sizeof(event_info));
  memset(event_notification_pending, 0, sizeof(event_notification_pending));
}

void high_speed_log_stop(void)
{
  uint8_t index;

  log_active = 0U;
  HighSpeedLogFinalizeCurrentEvent();

  if ((event_count == 0U) && (pre_sample_count != 0U))
  {
    current_event_index = 0U;
    memset(&event_info[0], 0, sizeof(event_info[0]));
    event_info[0].event_id = 1U;
    event_info[0].trigger_axis = 'N';
    event_info[0].trigger_reason = 0U;
    event_info[0].trigger_tick_ms = HAL_GetTick();
    event_info[0].trigger_control_sequence = control_sequence;
    event_info[0].trigger_sample_index = (uint16_t)(pre_sample_count - 1U);
    event_info[0].sample_count = pre_sample_count;
    event_pre_start[0] =
        (uint16_t)((pre_write_index + HIGH_SPEED_LOG_PRE_SAMPLES - pre_sample_count) %
                   HIGH_SPEED_LOG_PRE_SAMPLES);
    event_count = 1U;
  }

  for (index = 0U; index < event_count; index++)
  {
    HighSpeedLogRecalculateEventPeaks(index);
  }
}

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
                           float pitch_output)
{
  high_speed_log_sample_t sample;
  float roll_trigger_level;
  float pitch_trigger_level;
  uint8_t trigger_reason = 0U;
  uint8_t pre_event_index;
  int16_t roll_error_dps10;
  int16_t pitch_error_dps10;

  if (log_active == 0U)
  {
    return;
  }

  control_sequence++;
  sample.dt_us = HighSpeedLogMeasureDtUs();
  sample.roll_cdeg = HighSpeedLogScaleSigned(roll_deg, 100.0f);
  sample.pitch_cdeg = HighSpeedLogScaleSigned(pitch_deg, 100.0f);
  sample.roll_target_rate_dps10 = HighSpeedLogScaleSigned(roll_target_rate_dps, 10.0f);
  sample.pitch_target_rate_dps10 = HighSpeedLogScaleSigned(pitch_target_rate_dps, 10.0f);
  sample.gyro_x_dps10 = HighSpeedLogScaleSigned(gyro_x_dps, 10.0f);
  sample.gyro_y_dps10 = HighSpeedLogScaleSigned(gyro_y_dps, 10.0f);
  sample.roll_output_pwm10 = HighSpeedLogScaleSigned(roll_output, 10.0f);
  sample.pitch_output_pwm10 = HighSpeedLogScaleSigned(pitch_output, 10.0f);
  roll_error_dps10 = HighSpeedLogScaleSigned(roll_rate_error_dps, 10.0f);
  pitch_error_dps10 = HighSpeedLogScaleSigned(pitch_rate_error_dps, 10.0f);

  if (throttle < HIGH_SPEED_LOG_ARM_THROTTLE)
  {
    HighSpeedLogFinalizeCurrentEvent();
    pre_write_index = 0U;
    pre_sample_count = 0U;
    trigger_confirm_count = 0U;
    return;
  }

  if (capture_active != 0U)
  {
    if (current_event_sample_count < HIGH_SPEED_LOG_EVENT_SAMPLES)
    {
      event_samples[current_event_index][current_event_sample_count] = sample;
      HighSpeedLogUpdateEventPeaks(current_event_index,
                                   &sample,
                                   roll_error_dps10,
                                   pitch_error_dps10);
      current_event_sample_count++;
    }
    if (current_event_sample_count >= HIGH_SPEED_LOG_EVENT_SAMPLES)
    {
      HighSpeedLogFinalizeCurrentEvent();
    }
  }

  pre_event_index = (capture_active != 0U) ?
                    (uint8_t)(current_event_index + 1U) : event_count;
  if (pre_event_index < HIGH_SPEED_LOG_MAX_EVENTS)
  {
    event_samples[pre_event_index][pre_write_index] = sample;
    pre_write_index = (uint16_t)((pre_write_index + 1U) % HIGH_SPEED_LOG_PRE_SAMPLES);
    if (pre_sample_count < HIGH_SPEED_LOG_PRE_SAMPLES)
    {
      pre_sample_count++;
    }
  }

  if (capture_active != 0U)
  {
    return;
  }

  if (cooldown_sample_count != 0U)
  {
    cooldown_sample_count--;
    return;
  }

  if (pre_sample_count < HIGH_SPEED_LOG_PRE_SAMPLES)
  {
    return;
  }

  roll_trigger_level = fmaxf(fabsf(gyro_x_dps) / HIGH_SPEED_LOG_GYRO_TRIGGER_DPS,
                             fabsf(roll_rate_error_dps) / HIGH_SPEED_LOG_ERROR_TRIGGER_DPS);
  pitch_trigger_level = fmaxf(fabsf(gyro_y_dps) / HIGH_SPEED_LOG_GYRO_TRIGGER_DPS,
                              fabsf(pitch_rate_error_dps) / HIGH_SPEED_LOG_ERROR_TRIGGER_DPS);

  if (fabsf(gyro_x_dps) >= HIGH_SPEED_LOG_GYRO_TRIGGER_DPS)
  {
    trigger_reason |= HIGH_SPEED_LOG_REASON_ROLL_GYRO;
  }
  if (fabsf(gyro_y_dps) >= HIGH_SPEED_LOG_GYRO_TRIGGER_DPS)
  {
    trigger_reason |= HIGH_SPEED_LOG_REASON_PITCH_GYRO;
  }
  if (fabsf(roll_rate_error_dps) >= HIGH_SPEED_LOG_ERROR_TRIGGER_DPS)
  {
    trigger_reason |= HIGH_SPEED_LOG_REASON_ROLL_ERROR;
  }
  if (fabsf(pitch_rate_error_dps) >= HIGH_SPEED_LOG_ERROR_TRIGGER_DPS)
  {
    trigger_reason |= HIGH_SPEED_LOG_REASON_PITCH_ERROR;
  }

  if (trigger_reason != 0U)
  {
    trigger_confirm_count++;
    if (trigger_confirm_count >= HIGH_SPEED_LOG_TRIGGER_CONFIRM)
    {
      HighSpeedLogStartEvent((roll_trigger_level >= pitch_trigger_level) ? 'R' : 'P',
                             trigger_reason,
                             roll_error_dps10,
                             pitch_error_dps10);
      trigger_confirm_count = 0U;
    }
  }
  else
  {
    trigger_confirm_count = 0U;
  }
}

void high_speed_log_request_dump(void)
{
  if (event_count != 0U)
  {
    dump_pending = 1U;
  }
}

uint8_t high_speed_log_dump_requested(void)
{
  return dump_pending;
}

void high_speed_log_finish_dump(void)
{
  dump_pending = 0U;
  memset(event_notification_pending, 0, sizeof(event_notification_pending));
}

uint8_t high_speed_log_get_event_count(void)
{
  return event_count;
}

uint8_t high_speed_log_get_event_info(uint8_t event_index,
                                      high_speed_log_event_info_t *info)
{
  if ((info == NULL) || (event_index >= event_count))
  {
    return 0U;
  }
  *info = event_info[event_index];
  return 1U;
}

uint8_t high_speed_log_get_sample(uint8_t event_index,
                                  uint16_t sample_index,
                                  high_speed_log_sample_t *sample)
{
  if ((sample == NULL) || (event_index >= event_count) ||
      (sample_index >= event_info[event_index].sample_count))
  {
    return 0U;
  }
  *sample = *HighSpeedLogGetStoredSample(event_index, sample_index);
  return 1U;
}

uint8_t high_speed_log_get_pending_notification(high_speed_log_event_info_t *info)
{
  uint8_t index;
  uint8_t active_count = event_count + ((capture_active != 0U) ? 1U : 0U);

  if (info == NULL)
  {
    return 0U;
  }
  for (index = 0U; index < active_count; index++)
  {
    if (event_notification_pending[index] != 0U)
    {
      *info = event_info[index];
      return 1U;
    }
  }
  return 0U;
}

void high_speed_log_ack_notification(uint8_t event_id)
{
  if ((event_id > 0U) && (event_id <= HIGH_SPEED_LOG_MAX_EVENTS))
  {
    event_notification_pending[event_id - 1U] = 0U;
  }
}
