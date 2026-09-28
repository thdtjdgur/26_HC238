#include "sensor.h"

#include "debug.h"

#include <stdint.h>
#include <string.h>
#include <math.h>

extern SPI_HandleTypeDef hspi2;

#define IMU_CS_LOW()   HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET)
#define IMU_CS_HIGH()  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET)

#define LSM6DSR_CTRL1_XL_REG  0x10U
#define LSM6DSR_CTRL2_G_REG   0x11U
#define LSM6DSR_CTRL3_C_REG   0x12U
#define LSM6DSR_INT1_CTRL_REG 0x0DU
#define LSM6DSR_CTRL6_C_REG   0x15U
#define LSM6DSR_CTRL7_G_REG   0x16U
#define LSM6DSR_OUTX_L_G_REG  0x22U

#define LSM6DSR_ACCEL_1G_RAW            16384L
#define LSM6DSR_SETTLE_SAMPLES          16U
#define LSM6DSR_CALIBRATION_SAMPLES     256U
#define LSM6DSR_SETTLE_MAX_ATTEMPTS     64U
#define LSM6DSR_CALIBRATION_MAX_ATTEMPTS 320U
#define LSM6DSR_SPI_WRITE_RETRIES       4U
#define LSM6DSR_SPI_READY_TIMEOUT_MS    20U
#define LSM6DSR_DMA_TIMEOUT_MS          5U
#define LSM6DSR_GYRO_FILTER_SHIFT       2
#define LSM6DSR_ACCEL_FILTER_SHIFT      4
#define LSM6DSR_MOTION_BURST_LENGTH     12U
#define LSM6DSR_ACCEL_SENS_G_PER_LSB    0.000061f
#define LSM6DSR_GYRO_SENS_DPS_PER_LSB   0.00875f
#define LSM6DSR_DEFAULT_DT_SECONDS      (1.0f / 2000.0f)
#define LSM6DSR_MIN_DT_SECONDS          0.0001f
#define LSM6DSR_MAX_DT_SECONDS          0.0050f
#define LSM6DSR_RAD_TO_DEG              57.2957795f
#define LSM6DSR_COMP_TAU_SECONDS        0.5f
#define LSM6DSR_ACCEL_LPF_TAU_SECONDS   0.05f
#define LSM6DSR_GYRO_LPF_TAU_SECONDS    0.005f
#define LSM6DSR_ANGLE_LPF_TAU_SECONDS   0.010f
#define LSM6DSR_ACCEL_TRUST_FULL_G_ERR  0.25f
#define LSM6DSR_ACCEL_TRUST_ZERO_G_ERR  0.60f
#define LSM6DSR_ACCEL_TRUST_SLEW_SECONDS 0.05f
#define LSM6DSR_ZERO_TRIM_ALPHA         0.0005f
#define LSM6DSR_STILL_GYRO_DPS          1.0f
#define LSM6DSR_ACCEL_BIAS_X_RAW        (-183.425f)
#define LSM6DSR_ACCEL_BIAS_Y_RAW        (-179.0f)
#define LSM6DSR_ACCEL_BIAS_Z_RAW        337.375f
#define LSM6DSR_ACCEL_SCALE_X_RAW       15650.175f
#define LSM6DSR_ACCEL_SCALE_Y_RAW       15388.5f
#define LSM6DSR_ACCEL_SCALE_Z_RAW       15400.375f
#define LSM6DSR_ROLL_OFFSET_DEG         0.07f
#define LSM6DSR_PITCH_OFFSET_DEG        (-0.33f)

static uint8_t spi2_who_am_i_tx_buffer[32] __attribute__((aligned(32), section(".dma_buffer")));
static uint8_t spi2_who_am_i_rx_buffer[32] __attribute__((aligned(32), section(".dma_buffer")));
static volatile uint8_t spi2_who_am_i_ready = 0U;
static volatile uint8_t spi2_who_am_i_error = 0U;

volatile float sensor_gyro_x_dps = 0.0f;
volatile float sensor_gyro_y_dps = 0.0f;
volatile float sensor_gyro_z_dps = 0.0f;
volatile float sensor_accel_x_g = 0.0f;
volatile float sensor_accel_y_g = 0.0f;
volatile float sensor_accel_z_g = 0.0f;
volatile float sensor_roll_deg = 0.0f;
volatile float sensor_pitch_deg = 0.0f;
volatile float sensor_yaw_deg = 0.0f;
volatile float sensor_accel_norm_g = 0.0f;
volatile float sensor_accel_trust = 0.0f;
volatile int imuimu = 0;

typedef struct
{
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
} lsm6dsr_raw_data_t;

typedef struct
{
  int32_t gyro_x;
  int32_t gyro_y;
  int32_t gyro_z;
  int32_t accel_x;
  int32_t accel_y;
  int32_t accel_z;
} lsm6dsr_filtered_data_t;

typedef struct
{
  float roll_deg;
  float pitch_deg;
  float yaw_rel_deg;
  float accel_norm_g;
  float accel_lpf_x_g;
  float accel_lpf_y_g;
  float accel_lpf_z_g;
} lsm6dsr_attitude_t;

static lsm6dsr_raw_data_t motion_bias = {0};
static lsm6dsr_raw_data_t latest_motion_sample = {0};
static lsm6dsr_filtered_data_t filtered_motion = {0};
static lsm6dsr_attitude_t attitude = {0};
static float sensor_gyro_lpf_x_dps = 0.0f;
static float sensor_gyro_lpf_y_dps = 0.0f;
static float sensor_gyro_lpf_z_dps = 0.0f;
static float sensor_accel_lpf_x_g = 0.0f;
static float sensor_accel_lpf_y_g = 0.0f;
static float sensor_accel_lpf_z_g = 0.0f;
static float sensor_roll_lpf_deg = 0.0f;
static float sensor_pitch_lpf_deg = 0.0f;
static float sensor_yaw_lpf_deg = 0.0f;
static float accel_trust_state = 1.0f;
static uint8_t filter_initialized = 0U;
static uint8_t attitude_initialized = 0U;
static uint8_t sensor_output_filter_initialized = 0U;
static float attitude_roll_reference_deg = 0.0f;
static float attitude_pitch_reference_deg = 0.0f;
static volatile uint8_t spi2_motion_dma_busy = 0U;
static volatile uint8_t motion_sample_ready = 0U;
static volatile uint8_t motion_read_error = 0U;
static volatile uint32_t spi2_motion_dma_start_tick = 0U;
static volatile uint32_t motion_sample_timestamp_cycles = 0U;
static uint32_t previous_sample_timestamp_cycles = 0U;
static uint8_t sample_timestamp_initialized = 0U;
static float sensor_latest_dt_us = 0.0f;
static float sensor_gyro_input_x_dps = 0.0f;
static float sensor_gyro_input_y_dps = 0.0f;
static uint32_t timing_window_start_ms = 0U;
static uint32_t timing_window_sample_count = 0U;
static uint32_t timing_window_dt_count = 0U;
static uint32_t timing_window_clamped_count = 0U;
static float timing_window_dt_sum_us = 0.0f;
static float timing_window_dt_min_us = 0.0f;
static float timing_window_dt_max_us = 0.0f;
static uint32_t timing_report_sample_count = 0U;
static uint32_t timing_report_clamped_count = 0U;
static float timing_report_average_dt_us = 0.0f;
static float timing_report_minimum_dt_us = 0.0f;
static float timing_report_maximum_dt_us = 0.0f;
static HAL_StatusTypeDef LSM6DSR_WaitSpiReady(uint32_t timeout_ms);
static void LSM6DSR_RecoverDmaIfTimedOut(void);
static float LSM6DSR_LowPassStep(float previous_value, float input_value, float alpha);
static float LSM6DSR_CalibratedAccelRawToG(float raw_value, float bias, float scale);
static int32_t LSM6DSR_CalibratedAccelRawToIdealRaw(int16_t raw_value, float bias, float scale);

static HAL_StatusTypeDef LSM6DSR_WaitSpiReady(uint32_t timeout_ms)
{
  uint32_t start_tick = HAL_GetTick();

  while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)
  {
    if ((HAL_GetTick() - start_tick) >= timeout_ms)
    {
      return HAL_TIMEOUT;
    }
    HAL_Delay(1);
  }

  return HAL_OK;
}

static void LSM6DSR_RecoverDmaIfTimedOut(void)
{
  if ((spi2_motion_dma_busy != 0U) &&
      ((HAL_GetTick() - spi2_motion_dma_start_tick) > LSM6DSR_DMA_TIMEOUT_MS))
  {
    IMU_CS_HIGH();
    (void)HAL_SPI_Abort(&hspi2);
    spi2_motion_dma_busy = 0U;
    motion_read_error = 1U;
    spi2_who_am_i_error = 1U;
  }
}

static float LSM6DSR_LowPassStep(float previous_value, float input_value, float alpha)
{
  return previous_value + (alpha * (input_value - previous_value));
}

static float LSM6DSR_CalibratedAccelRawToG(float raw_value, float bias, float scale)
{
  return (raw_value - bias) / scale;
}

static int32_t LSM6DSR_CalibratedAccelRawToIdealRaw(int16_t raw_value, float bias, float scale)
{
  return (int32_t)(LSM6DSR_CalibratedAccelRawToG((float)raw_value, bias, scale) * (float)LSM6DSR_ACCEL_1G_RAW);
}

static void LSM6DSR_UpdateTimingStatistics(float actual_dt_us,
                                           uint8_t dt_valid,
                                           uint8_t dt_clamped)
{
  uint32_t now_ms = HAL_GetTick();

  if (timing_window_start_ms == 0U)
  {
    timing_window_start_ms = now_ms;
  }

  timing_window_sample_count++;

  if (dt_valid != 0U)
  {
    timing_window_dt_sum_us += actual_dt_us;
    if ((timing_window_dt_count == 0U) || (actual_dt_us < timing_window_dt_min_us))
    {
      timing_window_dt_min_us = actual_dt_us;
    }
    if ((timing_window_dt_count == 0U) || (actual_dt_us > timing_window_dt_max_us))
    {
      timing_window_dt_max_us = actual_dt_us;
    }
    timing_window_dt_count++;
  }

  if (dt_clamped != 0U)
  {
    timing_window_clamped_count++;
  }

  if ((now_ms - timing_window_start_ms) >= 1000U)
  {
    timing_report_sample_count = timing_window_sample_count;
    timing_report_clamped_count = timing_window_clamped_count;
    if (timing_window_dt_count != 0U)
    {
      timing_report_average_dt_us = timing_window_dt_sum_us / (float)timing_window_dt_count;
      timing_report_minimum_dt_us = timing_window_dt_min_us;
      timing_report_maximum_dt_us = timing_window_dt_max_us;
    }

    timing_window_start_ms = now_ms;
    timing_window_sample_count = 0U;
    timing_window_dt_count = 0U;
    timing_window_clamped_count = 0U;
    timing_window_dt_sum_us = 0.0f;
    timing_window_dt_min_us = 0.0f;
    timing_window_dt_max_us = 0.0f;
  }
}

static void SPI2_DMA_PrepareBuffers(void)
{
  //SCB_CleanDCache_by_Addr((uint32_t *)spi2_who_am_i_tx_buffer, (int32_t)sizeof(spi2_who_am_i_tx_buffer));
  //SCB_InvalidateDCache_by_Addr((uint32_t *)spi2_who_am_i_rx_buffer, (int32_t)sizeof(spi2_who_am_i_rx_buffer));
}

static void SPI2_DMA_CompleteBuffers(void)
{
  //SCB_InvalidateDCache_by_Addr((uint32_t *)spi2_who_am_i_rx_buffer, (int32_t)sizeof(spi2_who_am_i_rx_buffer));
}

static void LSM6DSR_ParseMotionBuffer(lsm6dsr_raw_data_t *motion, const uint8_t *motion_buffer)
{
  motion->gyro_x = (int16_t)((uint16_t)motion_buffer[1] << 8 | motion_buffer[0]);
  motion->gyro_y = (int16_t)((uint16_t)motion_buffer[3] << 8 | motion_buffer[2]);
  motion->gyro_z = (int16_t)((uint16_t)motion_buffer[5] << 8 | motion_buffer[4]);
  motion->accel_x = (int16_t)((uint16_t)motion_buffer[7] << 8 | motion_buffer[6]);
  motion->accel_y = (int16_t)((uint16_t)motion_buffer[9] << 8 | motion_buffer[8]);
  motion->accel_z = (int16_t)((uint16_t)motion_buffer[11] << 8 | motion_buffer[10]);
}

static HAL_StatusTypeDef LSM6DSR_WriteRegister(uint8_t reg, uint8_t value)
{
  uint8_t tx_buffer[2] = {reg & 0x7FU, value};
  uint32_t attempt;

  for (attempt = 0U; attempt < LSM6DSR_SPI_WRITE_RETRIES; ++attempt)
  {
    if (LSM6DSR_WaitSpiReady(LSM6DSR_SPI_READY_TIMEOUT_MS) != HAL_OK)
    {
      (void)HAL_SPI_Abort(&hspi2);
      HAL_Delay(1);
      continue;
    }

    IMU_CS_LOW();
    __NOP();
    __NOP();
    __NOP();

    if (HAL_SPI_Transmit(&hspi2, tx_buffer, 2U, 100U) == HAL_OK)
    {
      IMU_CS_HIGH();
      return HAL_OK;
    }

    IMU_CS_HIGH();
    HAL_Delay(1);
  }

  return HAL_ERROR;
}

static HAL_StatusTypeDef LSM6DSR_ReadRegistersBlocking(uint8_t reg, uint8_t *data, uint16_t length)
{
  uint8_t tx_buffer[LSM6DSR_MOTION_BURST_LENGTH + 1U] = {0};
  uint8_t rx_buffer[LSM6DSR_MOTION_BURST_LENGTH + 1U] = {0};

  if ((length == 0U) || (length > LSM6DSR_MOTION_BURST_LENGTH))
  {
    return HAL_ERROR;
  }

  tx_buffer[0] = reg | 0x80U;

  if (LSM6DSR_WaitSpiReady(LSM6DSR_SPI_READY_TIMEOUT_MS) != HAL_OK)
  {
    (void)HAL_SPI_Abort(&hspi2);
    return HAL_TIMEOUT;
  }

  IMU_CS_LOW();
  __NOP();
  __NOP();
  __NOP();

  if (HAL_SPI_TransmitReceive(&hspi2, tx_buffer, rx_buffer, length + 1U, 100U) != HAL_OK)
  {
    IMU_CS_HIGH();
    return HAL_ERROR;
  }

  IMU_CS_HIGH();
  memcpy(data, &rx_buffer[1], length);

  return HAL_OK;
}

static HAL_StatusTypeDef LSM6DSR_ReadMotionBlocking(lsm6dsr_raw_data_t *motion)
{
  uint8_t motion_buffer[LSM6DSR_MOTION_BURST_LENGTH];

  if (LSM6DSR_ReadRegistersBlocking(LSM6DSR_OUTX_L_G_REG, motion_buffer, sizeof(motion_buffer)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  LSM6DSR_ParseMotionBuffer(motion, motion_buffer);

  return HAL_OK;
}

static HAL_StatusTypeDef LSM6DSR_StartMotionReadDMA(void)
{
  LSM6DSR_RecoverDmaIfTimedOut();

  if (spi2_motion_dma_busy != 0U)
  {
    return HAL_BUSY;
  }

  spi2_who_am_i_tx_buffer[0] = LSM6DSR_OUTX_L_G_REG | 0x80U;
  memset(&spi2_who_am_i_tx_buffer[1], 0, LSM6DSR_MOTION_BURST_LENGTH);
  memset(spi2_who_am_i_rx_buffer, 0, LSM6DSR_MOTION_BURST_LENGTH + 1U);

  SPI2_DMA_PrepareBuffers();

  spi2_motion_dma_busy = 1U;
  spi2_motion_dma_start_tick = HAL_GetTick();
  motion_read_error = 0U;

  IMU_CS_LOW();
  __NOP();
  __NOP();
  __NOP();

  if (HAL_SPI_TransmitReceive_DMA(&hspi2, spi2_who_am_i_tx_buffer, spi2_who_am_i_rx_buffer, LSM6DSR_MOTION_BURST_LENGTH + 1U) != HAL_OK)
  {
    IMU_CS_HIGH();
    spi2_motion_dma_busy = 0U;
    motion_read_error = 1U;
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef LSM6DSR_CalibrateStationaryBias(void)
{
  lsm6dsr_raw_data_t sample;
  int32_t gyro_x_sum = 0;
  int32_t gyro_y_sum = 0;
  int32_t gyro_z_sum = 0;
  uint32_t index;
  uint32_t valid_samples = 0U;

  for (index = 0U; index < LSM6DSR_SETTLE_MAX_ATTEMPTS; ++index)
  {
    if (LSM6DSR_ReadMotionBlocking(&sample) != HAL_OK)
    {
      HAL_Delay(10);
      continue;
    }

    valid_samples++;
    HAL_Delay(10);

    if (valid_samples >= LSM6DSR_SETTLE_SAMPLES)
    {
      break;
    }
  }

  if (valid_samples < LSM6DSR_SETTLE_SAMPLES)
  {
    return HAL_ERROR;
  }

  valid_samples = 0U;

  for (index = 0U; index < LSM6DSR_CALIBRATION_MAX_ATTEMPTS; ++index)
  {
    if (LSM6DSR_ReadMotionBlocking(&sample) != HAL_OK)
    {
      HAL_Delay(10);
      continue;
    }

    gyro_x_sum += sample.gyro_x;
    gyro_y_sum += sample.gyro_y;
    gyro_z_sum += sample.gyro_z;
    valid_samples++;

    HAL_Delay(10);

    if (valid_samples >= LSM6DSR_CALIBRATION_SAMPLES)
    {
      break;
    }
  }

  if (valid_samples < LSM6DSR_CALIBRATION_SAMPLES)
  {
    return HAL_ERROR;
  }

  motion_bias.gyro_x = (int16_t)(gyro_x_sum / (int32_t)LSM6DSR_CALIBRATION_SAMPLES);
  motion_bias.gyro_y = (int16_t)(gyro_y_sum / (int32_t)LSM6DSR_CALIBRATION_SAMPLES);
  motion_bias.gyro_z = (int16_t)(gyro_z_sum / (int32_t)LSM6DSR_CALIBRATION_SAMPLES);
  motion_bias.accel_x = 0;
  motion_bias.accel_y = 0;
  motion_bias.accel_z = 0;

  return HAL_OK;
}

static void LSM6DSR_FilterMotion(const lsm6dsr_raw_data_t *raw_motion)
{
  int32_t gyro_x = (int32_t)raw_motion->gyro_x - motion_bias.gyro_x;
  int32_t gyro_y = (int32_t)raw_motion->gyro_y - motion_bias.gyro_y;
  int32_t gyro_z = (int32_t)raw_motion->gyro_z - motion_bias.gyro_z;
  int32_t accel_x = LSM6DSR_CalibratedAccelRawToIdealRaw(raw_motion->accel_x, LSM6DSR_ACCEL_BIAS_X_RAW, LSM6DSR_ACCEL_SCALE_X_RAW);
  int32_t accel_y = LSM6DSR_CalibratedAccelRawToIdealRaw(raw_motion->accel_y, LSM6DSR_ACCEL_BIAS_Y_RAW, LSM6DSR_ACCEL_SCALE_Y_RAW);
  int32_t accel_z = LSM6DSR_CalibratedAccelRawToIdealRaw(raw_motion->accel_z, LSM6DSR_ACCEL_BIAS_Z_RAW, LSM6DSR_ACCEL_SCALE_Z_RAW);

  if (filter_initialized == 0U)
  {
    filtered_motion.gyro_x = gyro_x;
    filtered_motion.gyro_y = gyro_y;
    filtered_motion.gyro_z = gyro_z;
    filtered_motion.accel_x = accel_x;
    filtered_motion.accel_y = accel_y;
    filtered_motion.accel_z = accel_z;
    filter_initialized = 1U;
    return;
  }

  filtered_motion.gyro_x += (gyro_x - filtered_motion.gyro_x) >> LSM6DSR_GYRO_FILTER_SHIFT;
  filtered_motion.gyro_y += (gyro_y - filtered_motion.gyro_y) >> LSM6DSR_GYRO_FILTER_SHIFT;
  filtered_motion.gyro_z += (gyro_z - filtered_motion.gyro_z) >> LSM6DSR_GYRO_FILTER_SHIFT;
  filtered_motion.accel_x += (accel_x - filtered_motion.accel_x) >> LSM6DSR_ACCEL_FILTER_SHIFT;
  filtered_motion.accel_y += (accel_y - filtered_motion.accel_y) >> LSM6DSR_ACCEL_FILTER_SHIFT;
  filtered_motion.accel_z += (accel_z - filtered_motion.accel_z) >> LSM6DSR_ACCEL_FILTER_SHIFT;
}

static void LSM6DSR_UpdateAttitude(const lsm6dsr_filtered_data_t *motion, float dt_seconds)
{
  float ax_g = (float)motion->accel_x * LSM6DSR_ACCEL_SENS_G_PER_LSB;
  float ay_g = (float)motion->accel_y * LSM6DSR_ACCEL_SENS_G_PER_LSB;
  float az_g = (float)motion->accel_z * LSM6DSR_ACCEL_SENS_G_PER_LSB;
  float gx_dps = (float)motion->gyro_x * LSM6DSR_GYRO_SENS_DPS_PER_LSB;
  float gy_dps = (float)motion->gyro_y * LSM6DSR_GYRO_SENS_DPS_PER_LSB;
  float gz_dps = (float)motion->gyro_z * LSM6DSR_GYRO_SENS_DPS_PER_LSB;
  float gyro_norm_dps = sqrtf((gx_dps * gx_dps) + (gy_dps * gy_dps) + (gz_dps * gz_dps));
  float accel_norm_g = sqrtf((ax_g * ax_g) + (ay_g * ay_g) + (az_g * az_g));
  float accel_error_g;
  float accel_trust_target;
  float accel_trust;
  float accel_trust_weight;
  float roll_acc_deg;
  float pitch_acc_deg;
  float output_roll_deg;
  float output_pitch_deg;
  float output_yaw_deg;
  float accel_trust_slew_step;
  float accel_correction_gain;
  float gyro_lpf_gain;
  float accel_lpf_gain;
  float angle_lpf_gain;

  accel_trust_slew_step = dt_seconds / LSM6DSR_ACCEL_TRUST_SLEW_SECONDS;
  accel_correction_gain = dt_seconds / (LSM6DSR_COMP_TAU_SECONDS + dt_seconds);
  gyro_lpf_gain = dt_seconds / (LSM6DSR_GYRO_LPF_TAU_SECONDS + dt_seconds);
  accel_lpf_gain = dt_seconds / (LSM6DSR_ACCEL_LPF_TAU_SECONDS + dt_seconds);
  angle_lpf_gain = dt_seconds / (LSM6DSR_ANGLE_LPF_TAU_SECONDS + dt_seconds);
  sensor_gyro_input_x_dps = gx_dps;
  sensor_gyro_input_y_dps = gy_dps;

  if (attitude_initialized == 0U)
  {
    attitude.accel_lpf_x_g = ax_g;
    attitude.accel_lpf_y_g = ay_g;
    attitude.accel_lpf_z_g = az_g;
    roll_acc_deg = atan2f(attitude.accel_lpf_y_g, attitude.accel_lpf_z_g) * LSM6DSR_RAD_TO_DEG;
    pitch_acc_deg = atan2f(-attitude.accel_lpf_x_g,
                           sqrtf((attitude.accel_lpf_y_g * attitude.accel_lpf_y_g) + (attitude.accel_lpf_z_g * attitude.accel_lpf_z_g))) * LSM6DSR_RAD_TO_DEG;
    attitude.roll_deg = roll_acc_deg;
    attitude.pitch_deg = pitch_acc_deg;
    attitude.yaw_rel_deg = 0.0f;
    attitude.accel_norm_g = accel_norm_g;
    attitude_roll_reference_deg = roll_acc_deg;
    attitude_pitch_reference_deg = pitch_acc_deg;
    sensor_gyro_lpf_x_dps = gx_dps;
    sensor_gyro_lpf_y_dps = gy_dps;
    sensor_gyro_lpf_z_dps = gz_dps;
    sensor_accel_lpf_x_g = ax_g;
    sensor_accel_lpf_y_g = ay_g;
    sensor_accel_lpf_z_g = az_g;
    sensor_roll_lpf_deg = attitude.roll_deg;
    sensor_pitch_lpf_deg = attitude.pitch_deg;
    sensor_yaw_lpf_deg = attitude.yaw_rel_deg;
    sensor_gyro_x_dps = sensor_gyro_lpf_x_dps;
    sensor_gyro_y_dps = sensor_gyro_lpf_y_dps;
    sensor_gyro_z_dps = sensor_gyro_lpf_z_dps;
    sensor_accel_x_g = sensor_accel_lpf_x_g;
    sensor_accel_y_g = sensor_accel_lpf_y_g;
    sensor_accel_z_g = sensor_accel_lpf_z_g;
    sensor_roll_deg = sensor_roll_lpf_deg - LSM6DSR_ROLL_OFFSET_DEG;
    sensor_pitch_deg = sensor_pitch_lpf_deg - LSM6DSR_PITCH_OFFSET_DEG;
    sensor_yaw_deg = sensor_yaw_lpf_deg;
    sensor_accel_norm_g = accel_norm_g;
    accel_trust_state = 1.0f;
    sensor_accel_trust = 1.0f;
    sensor_output_filter_initialized = 1U;
    attitude_initialized = 1U;
    return;
  }

  attitude.roll_deg += gx_dps * dt_seconds;
  attitude.pitch_deg += gy_dps * dt_seconds;
  attitude.yaw_rel_deg += gz_dps * dt_seconds;

  attitude.accel_lpf_x_g += accel_lpf_gain * (ax_g - attitude.accel_lpf_x_g);
  attitude.accel_lpf_y_g += accel_lpf_gain * (ay_g - attitude.accel_lpf_y_g);
  attitude.accel_lpf_z_g += accel_lpf_gain * (az_g - attitude.accel_lpf_z_g);

  roll_acc_deg = atan2f(attitude.accel_lpf_y_g, attitude.accel_lpf_z_g) * LSM6DSR_RAD_TO_DEG;
  pitch_acc_deg = atan2f(-attitude.accel_lpf_x_g,
                         sqrtf((attitude.accel_lpf_y_g * attitude.accel_lpf_y_g) + (attitude.accel_lpf_z_g * attitude.accel_lpf_z_g))) * LSM6DSR_RAD_TO_DEG;

  accel_error_g = fabsf(accel_norm_g - 1.0f);

  if (accel_error_g <= LSM6DSR_ACCEL_TRUST_FULL_G_ERR)
  {
    accel_trust_target = 1.0f;
  }
  else if (accel_error_g >= LSM6DSR_ACCEL_TRUST_ZERO_G_ERR)
  {
    accel_trust_target = 0.0f;
  }
  else
  {
    accel_trust_target = (LSM6DSR_ACCEL_TRUST_ZERO_G_ERR - accel_error_g) /
                         (LSM6DSR_ACCEL_TRUST_ZERO_G_ERR - LSM6DSR_ACCEL_TRUST_FULL_G_ERR);
  }

  if (accel_trust_state < accel_trust_target)
  {
    accel_trust_state += accel_trust_slew_step;
    if (accel_trust_state > accel_trust_target)
    {
      accel_trust_state = accel_trust_target;
    }
  }
  else if (accel_trust_state > accel_trust_target)
  {
    accel_trust_state -= accel_trust_slew_step;
    if (accel_trust_state < accel_trust_target)
    {
      accel_trust_state = accel_trust_target;
    }
  }

  accel_trust = accel_trust_state;

  accel_trust_weight = accel_trust * accel_trust;

  if (accel_trust_weight > 0.0f)
  {
    attitude.roll_deg += (roll_acc_deg - attitude.roll_deg) * (accel_correction_gain * accel_trust_weight);
    attitude.pitch_deg += (pitch_acc_deg - attitude.pitch_deg) * (accel_correction_gain * accel_trust_weight);
  }

  if (((accel_norm_g > 0.98f) && (accel_norm_g < 1.02f)) && (gyro_norm_dps < LSM6DSR_STILL_GYRO_DPS))
  {
    attitude_roll_reference_deg += (attitude.roll_deg - attitude_roll_reference_deg) * LSM6DSR_ZERO_TRIM_ALPHA;
    attitude_pitch_reference_deg += (attitude.pitch_deg - attitude_pitch_reference_deg) * LSM6DSR_ZERO_TRIM_ALPHA;
  }

  attitude.accel_norm_g = accel_norm_g;
  sensor_accel_norm_g = accel_norm_g;
  sensor_accel_trust = accel_trust;

  if (sensor_output_filter_initialized == 0U)
  {
    sensor_gyro_lpf_x_dps = gx_dps;
    sensor_gyro_lpf_y_dps = gy_dps;
    sensor_gyro_lpf_z_dps = gz_dps;
    sensor_accel_lpf_x_g = ax_g;
    sensor_accel_lpf_y_g = ay_g;
    sensor_accel_lpf_z_g = az_g;
    sensor_roll_lpf_deg = attitude.roll_deg;
    sensor_pitch_lpf_deg = attitude.pitch_deg;
    sensor_yaw_lpf_deg = attitude.yaw_rel_deg;
    sensor_output_filter_initialized = 1U;
  }
  else
  {
    sensor_gyro_lpf_x_dps = LSM6DSR_LowPassStep(sensor_gyro_lpf_x_dps, gx_dps, gyro_lpf_gain);
    sensor_gyro_lpf_y_dps = LSM6DSR_LowPassStep(sensor_gyro_lpf_y_dps, gy_dps, gyro_lpf_gain);
    sensor_gyro_lpf_z_dps = LSM6DSR_LowPassStep(sensor_gyro_lpf_z_dps, gz_dps, gyro_lpf_gain);
    sensor_accel_lpf_x_g = LSM6DSR_LowPassStep(sensor_accel_lpf_x_g, ax_g, accel_lpf_gain);
    sensor_accel_lpf_y_g = LSM6DSR_LowPassStep(sensor_accel_lpf_y_g, ay_g, accel_lpf_gain);
    sensor_accel_lpf_z_g = LSM6DSR_LowPassStep(sensor_accel_lpf_z_g, az_g, accel_lpf_gain);
    sensor_roll_lpf_deg = LSM6DSR_LowPassStep(sensor_roll_lpf_deg, attitude.roll_deg, angle_lpf_gain);
    sensor_pitch_lpf_deg = LSM6DSR_LowPassStep(sensor_pitch_lpf_deg, attitude.pitch_deg, angle_lpf_gain);
    sensor_yaw_lpf_deg = LSM6DSR_LowPassStep(sensor_yaw_lpf_deg, attitude.yaw_rel_deg, angle_lpf_gain);
  }

  output_roll_deg = sensor_roll_lpf_deg - LSM6DSR_ROLL_OFFSET_DEG;
  output_pitch_deg = sensor_pitch_lpf_deg - LSM6DSR_PITCH_OFFSET_DEG;
  output_yaw_deg = sensor_yaw_lpf_deg;
  sensor_gyro_x_dps = sensor_gyro_lpf_x_dps;
  sensor_gyro_y_dps = sensor_gyro_lpf_y_dps;
  sensor_gyro_z_dps = sensor_gyro_lpf_z_dps;
  sensor_accel_x_g = sensor_accel_lpf_x_g;
  sensor_accel_y_g = sensor_accel_lpf_y_g;
  sensor_accel_z_g = sensor_accel_lpf_z_g;
  sensor_roll_deg = output_roll_deg;
  sensor_pitch_deg = output_pitch_deg;
  sensor_yaw_deg = output_yaw_deg;
}

HAL_StatusTypeDef sensor_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
  {
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }

  IMU_CS_HIGH();
  HAL_Delay(35);

  if (LSM6DSR_WriteRegister(LSM6DSR_CTRL3_C_REG, 0x44U) != HAL_OK)
  {
    (void)uart1_printf("sensor init error: CTRL3_C\r\n");
    return HAL_ERROR;
  }

  if (LSM6DSR_WriteRegister(LSM6DSR_CTRL6_C_REG, 0x00U) != HAL_OK)
  {
    (void)uart1_printf("sensor init error: CTRL6_C\r\n");
    return HAL_ERROR;
  }

  if (LSM6DSR_WriteRegister(LSM6DSR_CTRL7_G_REG, 0x00U) != HAL_OK)
  {
    (void)uart1_printf("sensor init error: CTRL7_G\r\n");
    return HAL_ERROR;
  }

  if (LSM6DSR_WriteRegister(LSM6DSR_CTRL1_XL_REG, 0x90U) != HAL_OK)
  {
    (void)uart1_printf("sensor init error: CTRL1_XL\r\n");
    return HAL_ERROR;
  }

  if (LSM6DSR_WriteRegister(LSM6DSR_CTRL2_G_REG, 0x90U) != HAL_OK)
  {
    (void)uart1_printf("sensor init error: CTRL2_G\r\n");
    return HAL_ERROR;
  }

  if (LSM6DSR_CalibrateStationaryBias() != HAL_OK)
  {
    (void)uart1_printf("sensor init error: calibration\r\n");
    return HAL_ERROR;
  }

  if (LSM6DSR_WriteRegister(LSM6DSR_INT1_CTRL_REG, 0x02U) != HAL_OK)
  {
    (void)uart1_printf("sensor init error: INT1_CTRL\r\n");
    return HAL_ERROR;
  }

  return HAL_OK;
}

void sensor_process(void)
{
  lsm6dsr_raw_data_t motion_sample;
  uint32_t sample_timestamp_cycles;
  uint32_t cycle_delta;
  uint32_t primask;
  float actual_dt_seconds = LSM6DSR_DEFAULT_DT_SECONDS;
  float dt_seconds = LSM6DSR_DEFAULT_DT_SECONDS;
  uint8_t dt_valid = 0U;
  uint8_t dt_clamped = 0U;

  LSM6DSR_RecoverDmaIfTimedOut();

  if (motion_read_error != 0U)
  {
    motion_read_error = 0U;
  }

  if (motion_sample_ready == 0U)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  if (motion_sample_ready == 0U)
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    return;
  }

  motion_sample = latest_motion_sample;
  sample_timestamp_cycles = motion_sample_timestamp_cycles;
  motion_sample_ready = 0U;

  if (primask == 0U)
  {
    __enable_irq();
  }

  if ((sample_timestamp_initialized != 0U) && (SystemCoreClock != 0U))
  {
    cycle_delta = sample_timestamp_cycles - previous_sample_timestamp_cycles;
    actual_dt_seconds = (float)cycle_delta / (float)SystemCoreClock;
    dt_seconds = actual_dt_seconds;
    dt_valid = 1U;

    if (dt_seconds < LSM6DSR_MIN_DT_SECONDS)
    {
      dt_seconds = LSM6DSR_MIN_DT_SECONDS;
      dt_clamped = 1U;
    }
    else if (dt_seconds > LSM6DSR_MAX_DT_SECONDS)
    {
      dt_seconds = LSM6DSR_MAX_DT_SECONDS;
      dt_clamped = 1U;
    }
  }
  else
  {
    sample_timestamp_initialized = 1U;
  }

  previous_sample_timestamp_cycles = sample_timestamp_cycles;
  sensor_latest_dt_us = actual_dt_seconds * 1000000.0f;
  LSM6DSR_UpdateTimingStatistics(sensor_latest_dt_us, dt_valid, dt_clamped);
  LSM6DSR_FilterMotion(&motion_sample);
  LSM6DSR_UpdateAttitude(&filtered_motion, dt_seconds);
}

void sensor_get_attitude(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
  if (roll_deg != NULL)
  {
    *roll_deg = attitude.roll_deg;
  }

  if (pitch_deg != NULL)
  {
    *pitch_deg = attitude.pitch_deg;
  }

  if (yaw_deg != NULL)
  {
    *yaw_deg = attitude.yaw_rel_deg;
  }
}

void sensor_get_timing_debug(float *latest_dt_us,
                             uint32_t *sample_count,
                             float *average_dt_us,
                             float *minimum_dt_us,
                             float *maximum_dt_us,
                             uint32_t *clamped_count,
                             float *gyro_input_x_dps,
                             float *gyro_input_y_dps)
{
  if (latest_dt_us != NULL)
  {
    *latest_dt_us = sensor_latest_dt_us;
  }
  if (sample_count != NULL)
  {
    *sample_count = timing_report_sample_count;
  }
  if (average_dt_us != NULL)
  {
    *average_dt_us = timing_report_average_dt_us;
  }
  if (minimum_dt_us != NULL)
  {
    *minimum_dt_us = timing_report_minimum_dt_us;
  }
  if (maximum_dt_us != NULL)
  {
    *maximum_dt_us = timing_report_maximum_dt_us;
  }
  if (clamped_count != NULL)
  {
    *clamped_count = timing_report_clamped_count;
  }
  if (gyro_input_x_dps != NULL)
  {
    *gyro_input_x_dps = sensor_gyro_input_x_dps;
  }
  if (gyro_input_y_dps != NULL)
  {
    *gyro_input_y_dps = sensor_gyro_input_y_dps;
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI2)
  {
    SPI2_DMA_CompleteBuffers();
    LSM6DSR_ParseMotionBuffer(&latest_motion_sample, &spi2_who_am_i_rx_buffer[1]);
    motion_sample_timestamp_cycles = DWT->CYCCNT;
    IMU_CS_HIGH();
    spi2_motion_dma_busy = 0U;
    motion_sample_ready = 1U;
    spi2_who_am_i_ready = 1U;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI2)
  {
    IMU_CS_HIGH();
    spi2_motion_dma_busy = 0U;
    motion_read_error = 1U;
    spi2_who_am_i_error = 1U;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == SPI2_INT_Pin)
  {
    if (LSM6DSR_StartMotionReadDMA() == HAL_BUSY)
    {
      return;
    }

    if (spi2_motion_dma_busy == 0U)
    {
      motion_read_error = 1U;
      imuimu=1;
    }
  }
}
