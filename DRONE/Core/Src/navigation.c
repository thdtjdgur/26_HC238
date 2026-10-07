#include "navigation.h"

#include "main.h"

#include <math.h>
#include <stddef.h>

#define NAV_LATITUDE_LIMIT_DEG_1E7       900000000L
#define NAV_LONGITUDE_LIMIT_DEG_1E7     1800000000L
#define NAV_METERS_PER_DEG_1E7               0.01113195f
#define NAV_DEG_TO_RAD                       0.01745329252f
#define NAV_POSITION_KP_DEG_PER_M             0.90f
#define NAV_VELOCITY_KD_DEG_PER_MPS           1.20f
#define NAV_MAX_TILT_DEG                     12.0f
#define NAV_ARRIVAL_RADIUS_M                  1.50f
#define NAV_MAX_HORIZONTAL_ACCURACY_MM     5000U
#define NAV_MIN_COURSE_SPEED_MM_S           800
#define NAV_MAX_COURSE_ACCURACY_DEG_1E5 1500000U

/* Change these signs after a propeller-off body-axis test if required. */
#define NAV_ROLL_DIRECTION                     1.0f
#define NAV_PITCH_DIRECTION                   -1.0f

typedef struct
{
  volatile int32_t target_latitude_deg_1e7;
  volatile int32_t target_longitude_deg_1e7;
  int32_t current_latitude_deg_1e7;
  int32_t current_longitude_deg_1e7;
  int32_t north_velocity_mm_s;
  int32_t east_velocity_mm_s;
  float heading_deg;
  uint32_t last_update_tick_ms;
  volatile uint8_t target_valid;
  uint8_t position_valid;
  uint8_t heading_valid;
  navigation_status_t status;
} navigation_context_t;

static navigation_context_t navigation;

static float NavigationClamp(float value, float minimum, float maximum)
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

static float NavigationNormalizeHeading(float heading_deg)
{
  while (heading_deg >= 360.0f)
  {
    heading_deg -= 360.0f;
  }
  while (heading_deg < 0.0f)
  {
    heading_deg += 360.0f;
  }
  return heading_deg;
}

void navigation_init(void)
{
  navigation.target_latitude_deg_1e7 = 0;
  navigation.target_longitude_deg_1e7 = 0;
  navigation.current_latitude_deg_1e7 = 0;
  navigation.current_longitude_deg_1e7 = 0;
  navigation.north_velocity_mm_s = 0;
  navigation.east_velocity_mm_s = 0;
  navigation.heading_deg = 0.0f;
  navigation.last_update_tick_ms = 0U;
  navigation.target_valid = 0U;
  navigation.position_valid = 0U;
  navigation.heading_valid = 0U;
  navigation.status.north_error_m = 0.0f;
  navigation.status.east_error_m = 0.0f;
  navigation.status.distance_m = 0.0f;
  navigation.status.heading_deg = 0.0f;
  navigation.status.roll_target_deg = 0.0f;
  navigation.status.pitch_target_deg = 0.0f;
}

bool navigation_set_target(int32_t latitude_deg_1e7, int32_t longitude_deg_1e7)
{
  uint32_t primask;

  if ((latitude_deg_1e7 < -NAV_LATITUDE_LIMIT_DEG_1E7) ||
      (latitude_deg_1e7 > NAV_LATITUDE_LIMIT_DEG_1E7) ||
      (longitude_deg_1e7 < -NAV_LONGITUDE_LIMIT_DEG_1E7) ||
      (longitude_deg_1e7 > NAV_LONGITUDE_LIMIT_DEG_1E7))
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  navigation.target_latitude_deg_1e7 = latitude_deg_1e7;
  navigation.target_longitude_deg_1e7 = longitude_deg_1e7;
  navigation.target_valid = 1U;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return true;
}

void navigation_clear_target(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  navigation.target_valid = 0U;
  navigation.status.north_error_m = 0.0f;
  navigation.status.east_error_m = 0.0f;
  navigation.status.distance_m = 0.0f;
  navigation.status.roll_target_deg = 0.0f;
  navigation.status.pitch_target_deg = 0.0f;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void navigation_update_gnss(const gnss_pvt_t *pvt,
                            float absolute_heading_deg,
                            uint8_t absolute_heading_valid)
{
  if (pvt == NULL)
  {
    return;
  }

  navigation.position_valid = ((pvt->fix_ok != 0U) &&
                               (pvt->fix_type >= 3U) &&
                               (pvt->horizontal_accuracy_mm <= NAV_MAX_HORIZONTAL_ACCURACY_MM)) ? 1U : 0U;
  if (navigation.position_valid == 0U)
  {
    return;
  }

  navigation.current_latitude_deg_1e7 = pvt->latitude_deg_1e7;
  navigation.current_longitude_deg_1e7 = pvt->longitude_deg_1e7;
  navigation.north_velocity_mm_s = pvt->north_velocity_mm_s;
  navigation.east_velocity_mm_s = pvt->east_velocity_mm_s;

  if ((absolute_heading_valid != 0U) && isfinite(absolute_heading_deg))
  {
    navigation.heading_deg = NavigationNormalizeHeading(absolute_heading_deg);
    navigation.heading_valid = 1U;
  }
  else if ((pvt->ground_speed_mm_s >= NAV_MIN_COURSE_SPEED_MM_S) &&
           (pvt->heading_accuracy_deg_1e5 <= NAV_MAX_COURSE_ACCURACY_DEG_1E5))
  {
    navigation.heading_deg = NavigationNormalizeHeading((float)pvt->heading_motion_deg_1e5 * 0.00001f);
    navigation.heading_valid = 1U;
  }
  else
  {
    navigation.heading_valid = 0U;
  }

  navigation.last_update_tick_ms = HAL_GetTick();
}

bool navigation_is_ready(uint32_t maximum_age_ms)
{
  if ((navigation.target_valid == 0U) ||
      (navigation.position_valid == 0U) ||
      (navigation.heading_valid == 0U))
  {
    return false;
  }

  return ((uint32_t)(HAL_GetTick() - navigation.last_update_tick_ms) <= maximum_age_ms);
}

bool navigation_compute_attitude_targets(float *roll_target_deg,
                                         float *pitch_target_deg)
{
  int64_t latitude_delta;
  int64_t longitude_delta;
  int32_t target_latitude_deg_1e7;
  int32_t target_longitude_deg_1e7;
  uint32_t primask;
  float latitude_rad;
  float north_error_m;
  float east_error_m;
  float north_velocity_mps;
  float east_velocity_mps;
  float north_tilt_command_deg;
  float east_tilt_command_deg;
  float heading_rad;
  float forward_tilt_command_deg;
  float right_tilt_command_deg;
  float roll_command_deg;
  float pitch_command_deg;
  float distance_m;
  navigation_status_t updated_status;

  if ((roll_target_deg == NULL) || (pitch_target_deg == NULL) ||
      (!navigation_is_ready(1000U)))
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  target_latitude_deg_1e7 = navigation.target_latitude_deg_1e7;
  target_longitude_deg_1e7 = navigation.target_longitude_deg_1e7;
  if (primask == 0U)
  {
    __enable_irq();
  }

  latitude_delta = target_latitude_deg_1e7 - navigation.current_latitude_deg_1e7;
  longitude_delta = target_longitude_deg_1e7 - navigation.current_longitude_deg_1e7;
  if (longitude_delta > NAV_LONGITUDE_LIMIT_DEG_1E7)
  {
    longitude_delta -= (2LL * NAV_LONGITUDE_LIMIT_DEG_1E7);
  }
  else if (longitude_delta < -NAV_LONGITUDE_LIMIT_DEG_1E7)
  {
    longitude_delta += (2LL * NAV_LONGITUDE_LIMIT_DEG_1E7);
  }
  latitude_rad = (float)navigation.current_latitude_deg_1e7 * 0.0000001f * NAV_DEG_TO_RAD;
  north_error_m = (float)latitude_delta * NAV_METERS_PER_DEG_1E7;
  east_error_m = (float)longitude_delta * NAV_METERS_PER_DEG_1E7 * cosf(latitude_rad);
  north_velocity_mps = (float)navigation.north_velocity_mm_s * 0.001f;
  east_velocity_mps = (float)navigation.east_velocity_mm_s * 0.001f;
  distance_m = sqrtf((north_error_m * north_error_m) + (east_error_m * east_error_m));

  if (distance_m <= NAV_ARRIVAL_RADIUS_M)
  {
    north_error_m = 0.0f;
    east_error_m = 0.0f;
  }

  north_tilt_command_deg = (NAV_POSITION_KP_DEG_PER_M * north_error_m) -
                           (NAV_VELOCITY_KD_DEG_PER_MPS * north_velocity_mps);
  east_tilt_command_deg = (NAV_POSITION_KP_DEG_PER_M * east_error_m) -
                          (NAV_VELOCITY_KD_DEG_PER_MPS * east_velocity_mps);

  heading_rad = navigation.heading_deg * NAV_DEG_TO_RAD;
  forward_tilt_command_deg = (north_tilt_command_deg * cosf(heading_rad)) +
                             (east_tilt_command_deg * sinf(heading_rad));
  right_tilt_command_deg = (-north_tilt_command_deg * sinf(heading_rad)) +
                            (east_tilt_command_deg * cosf(heading_rad));

  roll_command_deg = NavigationClamp(NAV_ROLL_DIRECTION * right_tilt_command_deg,
                                     -NAV_MAX_TILT_DEG,
                                      NAV_MAX_TILT_DEG);
  pitch_command_deg = NavigationClamp(NAV_PITCH_DIRECTION * forward_tilt_command_deg,
                                      -NAV_MAX_TILT_DEG,
                                       NAV_MAX_TILT_DEG);

  updated_status.north_error_m = north_error_m;
  updated_status.east_error_m = east_error_m;
  updated_status.distance_m = distance_m;
  updated_status.heading_deg = navigation.heading_deg;
  updated_status.roll_target_deg = roll_command_deg;
  updated_status.pitch_target_deg = pitch_command_deg;
  primask = __get_PRIMASK();
  __disable_irq();
  navigation.status = updated_status;
  if (primask == 0U)
  {
    __enable_irq();
  }
  *roll_target_deg = roll_command_deg;
  *pitch_target_deg = pitch_command_deg;
  return true;
}

void navigation_get_status(navigation_status_t *status)
{
  if (status != NULL)
  {
    *status = navigation.status;
  }
}
