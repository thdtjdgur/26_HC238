#ifndef __NAVIGATION_H__
#define __NAVIGATION_H__

#include "gnss.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  float north_error_m;
  float east_error_m;
  float distance_m;
  float heading_deg;
  float roll_target_deg;
  float pitch_target_deg;
} navigation_status_t;

void navigation_init(void);
bool navigation_set_target(int32_t latitude_deg_1e7, int32_t longitude_deg_1e7);
void navigation_clear_target(void);
void navigation_update_gnss(const gnss_pvt_t *pvt,
                            float absolute_heading_deg,
                            uint8_t absolute_heading_valid);
bool navigation_is_ready(uint32_t maximum_age_ms);
bool navigation_compute_attitude_targets(float *roll_target_deg,
                                         float *pitch_target_deg);
void navigation_get_status(navigation_status_t *status);

#endif
