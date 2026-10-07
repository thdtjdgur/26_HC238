#include "debug.h"
#include "flight_control.h"
#include "gnss.h"
#include "high_speed_log.h"
#include "motor.h"
#include "navigation.h"
#include "oled.h"
#include "main.h"
#include "sensor.h"
#include "uart_bridge.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim3;
extern uint32_t user_step_throttle_compare;
extern float battery_voltage;
extern int8_t battery_percent;
extern uint8_t battery_valid;

#define UART1_DMA_TX_QUEUE_LENGTH 8U
#define UART1_DMA_TX_BUFFER_SIZE  160U
#define UART1_DMA_CACHE_LINE_SIZE 32U
#define UART1_DMA_RX_BUFFER_SIZE  16U
#define UART1_TRIGGER_HOLD_MS      1000U
#define UART6_DMA_TX_QUEUE_LENGTH 8U
#define UART6_DMA_TX_BUFFER_SIZE  384U
#define UART6_DMA_RX_BUFFER_SIZE  64U
#define UART6_FULL_TELEMETRY_ENABLED 1U
#define UART6_HIGH_SPEED_DUMP_INTERVAL_MS 50U
#define UART6_HIGH_SPEED_DUMP_BATCH_SAMPLES 2U

typedef struct
{
  uint8_t data[UART1_DMA_TX_BUFFER_SIZE];
  uint16_t length;
  uint8_t reserved[30];
} uart1_dma_tx_entry_t;

typedef struct
{
  uint8_t data[UART6_DMA_TX_BUFFER_SIZE];
  uint16_t length;
  uint8_t reserved[30];
} uart6_dma_tx_entry_t;

static uart1_dma_tx_entry_t uart1_dma_tx_queue[UART1_DMA_TX_QUEUE_LENGTH]
  __attribute__((aligned(UART1_DMA_CACHE_LINE_SIZE), section(".dma_buffer")));
static uart6_dma_tx_entry_t uart6_dma_tx_queue[UART6_DMA_TX_QUEUE_LENGTH]
  __attribute__((aligned(UART1_DMA_CACHE_LINE_SIZE), section(".dma_buffer")));
static uint8_t uart1_dma_rx_buffer[UART1_DMA_RX_BUFFER_SIZE]
  __attribute__((aligned(UART1_DMA_CACHE_LINE_SIZE), section(".dma_buffer")));
static uint8_t uart6_dma_rx_buffer[UART6_DMA_RX_BUFFER_SIZE]
  __attribute__((aligned(UART1_DMA_CACHE_LINE_SIZE), section(".dma_buffer")));
static volatile uint8_t uart1_dma_tx_busy = 0U;
static volatile uint8_t uart6_dma_tx_busy = 0U;
static volatile uint8_t uart1_dma_tx_head = 0U;
static volatile uint8_t uart1_dma_tx_tail = 0U;
static volatile uint8_t uart1_dma_tx_count = 0U;
static volatile uint8_t uart6_dma_tx_head = 0U;
static volatile uint8_t uart6_dma_tx_tail = 0U;
static volatile uint8_t uart6_dma_tx_count = 0U;
volatile float uart1_rx_float_value = 0.0f;
volatile float uart6_rx_float_value = 0.0f;
volatile uint8_t debug_uart_update_flag = 0U;
volatile uint8_t debug_uart6_update_flag = 0U;
volatile uint8_t debug_oled_update_flag = 0U;
volatile uint8_t debug_oled_tick_divider = 0U;
volatile uint32_t main_flag = 0U;
static volatile uint8_t uart1_trigger_active = 0U;
static volatile uint32_t uart1_trigger_start_tick = 0U;
static volatile uint8_t uart6_start_received = 0U;
static volatile uint8_t uart6_arm_requested = 0U;
static volatile uint8_t uart6_disarm_requested = 0U;
static volatile uint8_t debug_bridge_mode_active = 0U;

typedef enum
{
  HIGH_SPEED_DUMP_IDLE = 0,
  HIGH_SPEED_DUMP_BEGIN,
  HIGH_SPEED_DUMP_EVENT_BEGIN,
  HIGH_SPEED_DUMP_SAMPLE,
  HIGH_SPEED_DUMP_EVENT_END,
  HIGH_SPEED_DUMP_END
} high_speed_dump_phase_t;

static high_speed_dump_phase_t high_speed_dump_phase = HIGH_SPEED_DUMP_IDLE;
static uint8_t high_speed_dump_event_index = 0U;
static uint16_t high_speed_dump_sample_index = 0U;
static uint32_t high_speed_dump_last_send_tick = 0U;

static HAL_StatusTypeDef UART6_DMATxEnqueue(const uint8_t *data, uint16_t length);

/* UART6 control protocol:
 *   START
 *   MODE,ANGLE | MODE,0 | MODE,GPS | MODE,1
 *   CMD,<pwm>,<roll_deg>,<pitch_deg>,<yaw_rate_dps>
 *   GPS,<latitude_deg>,<longitude_deg>,<pwm>
 *   HEARTBEAT | STATUS | NAVSTATUS | ARM | DISARM
 */
static bool DebugParseAngleCommand(const char *text,
                                   uint16_t *throttle_compare,
                                   float *roll_angle_deg,
                                   float *pitch_angle_deg,
                                   float *yaw_rate_dps)
{
  const char *cursor;
  char *parse_end;
  unsigned long throttle;

  if ((text == NULL) || (throttle_compare == NULL) ||
      (roll_angle_deg == NULL) || (pitch_angle_deg == NULL) ||
      (yaw_rate_dps == NULL) || (strncmp(text, "CMD,", 4U) != 0))
  {
    return false;
  }

  cursor = &text[4];
  throttle = strtoul(cursor, &parse_end, 10);
  if ((parse_end == cursor) || (*parse_end != ',') || (throttle > UINT16_MAX))
  {
    return false;
  }
  cursor = parse_end + 1;

  *roll_angle_deg = strtof(cursor, &parse_end);
  if ((parse_end == cursor) || (*parse_end != ','))
  {
    return false;
  }
  cursor = parse_end + 1;

  *pitch_angle_deg = strtof(cursor, &parse_end);
  if ((parse_end == cursor) || (*parse_end != ','))
  {
    return false;
  }
  cursor = parse_end + 1;

  *yaw_rate_dps = strtof(cursor, &parse_end);
  if ((parse_end == cursor) || (*parse_end != '\0'))
  {
    return false;
  }

  *throttle_compare = (uint16_t)throttle;
  return true;
}

static bool DebugParseGpsCommand(const char *text,
                                 int32_t *latitude_deg_1e7,
                                 int32_t *longitude_deg_1e7,
                                 uint16_t *throttle_compare)
{
  const char *cursor;
  char *parse_end;
  double latitude_deg;
  double longitude_deg;
  unsigned long throttle;

  if ((text == NULL) || (latitude_deg_1e7 == NULL) ||
      (longitude_deg_1e7 == NULL) || (throttle_compare == NULL) ||
      (strncmp(text, "GPS,", 4U) != 0))
  {
    return false;
  }

  cursor = &text[4];
  latitude_deg = strtod(cursor, &parse_end);
  if ((parse_end == cursor) || (*parse_end != ','))
  {
    return false;
  }
  cursor = parse_end + 1;

  longitude_deg = strtod(cursor, &parse_end);
  if ((parse_end == cursor) || (*parse_end != ','))
  {
    return false;
  }
  cursor = parse_end + 1;

  throttle = strtoul(cursor, &parse_end, 10);
  if ((parse_end == cursor) || (*parse_end != '\0'))
  {
    return false;
  }

  if ((latitude_deg < -90.0) || (latitude_deg > 90.0) ||
      (longitude_deg < -180.0) || (longitude_deg > 180.0) ||
      (throttle > UINT16_MAX))
  {
    return false;
  }

  *latitude_deg_1e7 = (int32_t)((latitude_deg * 10000000.0) +
                                ((latitude_deg >= 0.0) ? 0.5 : -0.5));
  *longitude_deg_1e7 = (int32_t)((longitude_deg * 10000000.0) +
                                 ((longitude_deg >= 0.0) ? 0.5 : -0.5));
  *throttle_compare = (uint16_t)throttle;
  return true;
}

static void UART1_DMARxStart(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
                                   uart1_dma_rx_buffer,
                                   UART1_DMA_RX_BUFFER_SIZE) == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}

static void UART6_DMARxStart(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart6,
                                   uart6_dma_rx_buffer,
                                   UART6_DMA_RX_BUFFER_SIZE) == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
  }
}

static void UART6_DMATxRespond(uint16_t length)
{
  static const uint8_t started_text[] = "STARTED\r\n";
  static const uint8_t retry_text[] = "RETRY\r\n";
  static const uint8_t command_ack_text[] = "CMD_OK\r\n";
  static const uint8_t command_error_text[] = "CMD_ERROR\r\n";
  static const uint8_t mode_ack_text[] = "MODE_OK\r\n";
  static const uint8_t gps_ack_text[] = "GPS_OK\r\n";
  static const uint8_t heartbeat_ack_text[] = "HEARTBEAT_OK\r\n";
  static const uint8_t arm_requested_text[] = "ARM_REQUESTED\r\n";
  static const uint8_t disarm_requested_text[] = "DISARM_REQUESTED\r\n";
  const uint8_t *tx_data = command_error_text;
  uint16_t tx_length = (uint16_t)(sizeof(command_error_text) - 1U);
  char rx_text[UART6_DMA_RX_BUFFER_SIZE];
  char status_text[128];
  uint16_t angle_throttle_compare;
  int32_t target_latitude_deg_1e7;
  int32_t target_longitude_deg_1e7;
  uint16_t gps_throttle_compare;
  float roll_angle_deg;
  float pitch_angle_deg;
  float yaw_rate_dps;
  int status_length;
  navigation_status_t navigation_status;
  int north_error_cm;
  int east_error_cm;
  int distance_cm;
  int heading_cdeg;
  int roll_target_cdeg;
  int pitch_target_cdeg;

  while ((length > 0U) &&
         ((uart6_dma_rx_buffer[length - 1U] == '\r') || (uart6_dma_rx_buffer[length - 1U] == '\n')))
  {
    length--;
  }

  if (uart6_start_received == 0U)
  {
    if ((length == 5U) && (memcmp(uart6_dma_rx_buffer, "START", 5U) == 0))
    {
      uart6_start_received = 1U;
      tx_data = started_text;
      tx_length = (uint16_t)(sizeof(started_text) - 1U);
    }
    else
    {
      tx_data = retry_text;
      tx_length = (uint16_t)(sizeof(retry_text) - 1U);
    }
  }
  else
  {
    if (length >= UART6_DMA_RX_BUFFER_SIZE)
    {
      length = UART6_DMA_RX_BUFFER_SIZE - 1U;
    }

    memcpy(rx_text, uart6_dma_rx_buffer, length);
    rx_text[length] = '\0';

    if (strcmp(rx_text, "ARM") == 0)
    {
      uart6_arm_requested = 1U;
      tx_data = arm_requested_text;
      tx_length = (uint16_t)(sizeof(arm_requested_text) - 1U);
    }
    else if (strcmp(rx_text, "DISARM") == 0)
    {
      uart6_disarm_requested = 1U;
      tx_data = disarm_requested_text;
      tx_length = (uint16_t)(sizeof(disarm_requested_text) - 1U);
    }
    else if (strcmp(rx_text, "STATUS") == 0)
    {
      status_length = snprintf(status_text,
                               sizeof(status_text),
                               "STATE,%s,%s,%s\r\n",
                               flight_control_state_text(),
                               flight_control_mode_text(),
                               flight_control_failsafe_text());
      if ((status_length > 0) && (status_length < (int)sizeof(status_text)))
      {
        (void)UART6_DMATxEnqueue((const uint8_t *)status_text, (uint16_t)status_length);
      }
      return;
    }
    else if (((strcmp(rx_text, "MODE,ANGLE") == 0) ||
              (strcmp(rx_text, "MODE,0") == 0)) &&
             flight_control_set_mode(FLIGHT_MODE_ANGLE))
    {
      tx_data = mode_ack_text;
      tx_length = (uint16_t)(sizeof(mode_ack_text) - 1U);
    }
    else if (((strcmp(rx_text, "MODE,GPS") == 0) ||
              (strcmp(rx_text, "MODE,1") == 0)) &&
             flight_control_set_mode(FLIGHT_MODE_GPS))
    {
      tx_data = mode_ack_text;
      tx_length = (uint16_t)(sizeof(mode_ack_text) - 1U);
    }
    else if ((strcmp(rx_text, "HEARTBEAT") == 0) &&
             flight_control_refresh_command())
    {
      tx_data = heartbeat_ack_text;
      tx_length = (uint16_t)(sizeof(heartbeat_ack_text) - 1U);
    }
    else if (strcmp(rx_text, "NAVSTATUS") == 0)
    {
      navigation_get_status(&navigation_status);
      north_error_cm = (int)(navigation_status.north_error_m * 100.0f);
      east_error_cm = (int)(navigation_status.east_error_m * 100.0f);
      distance_cm = (int)(navigation_status.distance_m * 100.0f);
      heading_cdeg = (int)(navigation_status.heading_deg * 100.0f);
      roll_target_cdeg = (int)(navigation_status.roll_target_deg * 100.0f);
      pitch_target_cdeg = (int)(navigation_status.pitch_target_deg * 100.0f);
      status_length = snprintf(status_text,
                               sizeof(status_text),
                               "NAV,Ncm=%d,Ecm=%d,Dcm=%d,Hcdeg=%d,Rcdeg=%d,Pcdeg=%d\r\n",
                               north_error_cm,
                               east_error_cm,
                               distance_cm,
                               heading_cdeg,
                               roll_target_cdeg,
                               pitch_target_cdeg);
      if ((status_length > 0) && (status_length < (int)sizeof(status_text)))
      {
        (void)UART6_DMATxEnqueue((const uint8_t *)status_text, (uint16_t)status_length);
      }
      return;
    }
    else if (DebugParseGpsCommand(rx_text,
                                  &target_latitude_deg_1e7,
                                  &target_longitude_deg_1e7,
                                  &gps_throttle_compare) &&
             flight_control_set_gps_target(target_latitude_deg_1e7,
                                           target_longitude_deg_1e7,
                                           gps_throttle_compare))
    {
      tx_data = gps_ack_text;
      tx_length = (uint16_t)(sizeof(gps_ack_text) - 1U);
    }
    else
    {
      if (DebugParseAngleCommand(rx_text,
                                 &angle_throttle_compare,
                                 &roll_angle_deg,
                                 &pitch_angle_deg,
                                 &yaw_rate_dps) &&
          flight_control_set_command(angle_throttle_compare,
                                     roll_angle_deg,
                                     pitch_angle_deg,
                                     yaw_rate_dps))
      {
        uart6_rx_float_value = (float)angle_throttle_compare;
        tx_data = command_ack_text;
        tx_length = (uint16_t)(sizeof(command_ack_text) - 1U);
      }
    }
  }

  (void)UART6_DMATxEnqueue(tx_data, tx_length);
}

static void UART1_DMARxStoreFloat(uint16_t length)
{
  char rx_text[UART1_DMA_RX_BUFFER_SIZE];
  char *parse_end;
  long trigger_index;
  float target_roll_rate_dps;

  if ((length == 0U) || (length >= UART1_DMA_RX_BUFFER_SIZE))
  {
    return;
  }

  memcpy(rx_text, uart1_dma_rx_buffer, length);
  rx_text[length] = '\0';

  parse_end = rx_text;
  while (*parse_end == ' ')
  {
    parse_end++;
  }

  if (*parse_end != 'T')
  {
    return;
  }

  parse_end++;

  while (*parse_end == ' ')
  {
    parse_end++;
  }

  if (*parse_end != ',')
  {
    return;
  }

  parse_end++;

  while (*parse_end == ' ')
  {
    parse_end++;
  }

  trigger_index = strtol(parse_end, &parse_end, 10);
  if ((*parse_end != '\0') && (*parse_end != ' ') && (*parse_end != '\r') && (*parse_end != '\n'))
  {
    return;
  }

  target_roll_rate_dps = 0.0f;

  if (trigger_index == 1)
  {
    target_roll_rate_dps = 5.0f;
  }
  else if (trigger_index == 2)
  {
    target_roll_rate_dps = 10.0f;
  }
  else if (trigger_index == 3)
  {
    target_roll_rate_dps = 15.0f;
  }
  else if (trigger_index != 0)
  {
    return;
  }

  uart1_rx_float_value = target_roll_rate_dps;
  motor_set_rate_targets(target_roll_rate_dps, 0.0f, 0.0f);

  if (target_roll_rate_dps > 0.0f)
  {
    uart1_trigger_active = 1U;
    uart1_trigger_start_tick = HAL_GetTick();
  }
  else
  {
    uart1_trigger_active = 0U;
  }

  debug_oled_update_flag = 1U;
}

static int DebugAbs(int value)
{
  return (value < 0) ? -value : value;
}

static int DebugSignScaledTenths(float value)
{
  return (value < 0.0f) ? -DebugAbs((int)(value * 10.0f)) : DebugAbs((int)(value * 10.0f));
}

static int DebugIntegerPartFromScaled(int scaled_value, int scale)
{
  if ((scaled_value < 0) && (DebugAbs(scaled_value) < scale))
  {
    return -0;
  }

  return scaled_value / scale;
}

static uint32_t UART1_DMATxEnterCritical(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void UART1_DMATxExitCritical(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

// static void UART1_DMATxCleanCache(const void *address, uint32_t length)
// {
//   //uintptr_t aligned_address = ((uintptr_t)address) & ~(uintptr_t)(UART1_DMA_CACHE_LINE_SIZE - 1U);
//   //uint32_t aligned_length = (uint32_t)(((uintptr_t)address + length + (UART1_DMA_CACHE_LINE_SIZE - 1U)) - aligned_address);

//   //SCB_CleanDCache_by_Addr((uint32_t *)aligned_address, (int32_t)aligned_length);
// }

static void UART1_DMATxStartNext(void)
{
  uint32_t primask;
  uint8_t queue_tail;
  HAL_StatusTypeDef status;

  primask = UART1_DMATxEnterCritical();

  if ((uart1_dma_tx_busy != 0U) || (uart1_dma_tx_count == 0U))
  {
    UART1_DMATxExitCritical(primask);
    return;
  }

  queue_tail = uart1_dma_tx_tail;
  uart1_dma_tx_busy = 1U;
  UART1_DMATxExitCritical(primask);

  //UART1_DMATxCleanCache(uart1_dma_tx_queue[queue_tail].data, UART1_DMA_TX_BUFFER_SIZE);

  status = HAL_UART_Transmit_DMA(&huart1,
                                 uart1_dma_tx_queue[queue_tail].data,
                                 uart1_dma_tx_queue[queue_tail].length);

  if (status != HAL_OK)
  {
    HAL_UART_Abort(&huart1);

    primask = UART1_DMATxEnterCritical();
    uart1_dma_tx_busy = 0U;
    UART1_DMATxExitCritical(primask);
  }
}

static HAL_StatusTypeDef UART1_DMATxEnqueue(const uint8_t *data, uint16_t length)
{
  uint32_t primask;
  uint8_t queue_head;

  if ((data == NULL) || (length == 0U) || (length > UART1_DMA_TX_BUFFER_SIZE))
  {
    return HAL_ERROR;
  }

  primask = UART1_DMATxEnterCritical();

  if (uart1_dma_tx_count >= UART1_DMA_TX_QUEUE_LENGTH)
  {
    UART1_DMATxExitCritical(primask);
    return HAL_BUSY;
  }

  queue_head = uart1_dma_tx_head;
  memcpy(uart1_dma_tx_queue[queue_head].data, data, length);
  uart1_dma_tx_queue[queue_head].length = length;
  uart1_dma_tx_head = (uint8_t)((queue_head + 1U) % UART1_DMA_TX_QUEUE_LENGTH);
  uart1_dma_tx_count++;

  UART1_DMATxExitCritical(primask);

  UART1_DMATxStartNext();

  return HAL_OK;
}

static void UART6_DMATxStartNext(void)
{
  uint32_t primask;
  uint8_t queue_tail;
  HAL_StatusTypeDef status;

  primask = UART1_DMATxEnterCritical();

  if ((uart6_dma_tx_busy != 0U) || (uart6_dma_tx_count == 0U))
  {
    UART1_DMATxExitCritical(primask);
    return;
  }

  queue_tail = uart6_dma_tx_tail;
  uart6_dma_tx_busy = 1U;
  UART1_DMATxExitCritical(primask);

  status = HAL_UART_Transmit_DMA(&huart6,
                                 uart6_dma_tx_queue[queue_tail].data,
                                 uart6_dma_tx_queue[queue_tail].length);

  if (status != HAL_OK)
  {
    HAL_UART_Abort(&huart6);

    primask = UART1_DMATxEnterCritical();
    uart6_dma_tx_busy = 0U;
    UART1_DMATxExitCritical(primask);
  }
}

static HAL_StatusTypeDef UART6_DMATxEnqueue(const uint8_t *data, uint16_t length)
{
  uint32_t primask;
  uint8_t queue_head;

  if ((data == NULL) || (length == 0U) || (length > UART6_DMA_TX_BUFFER_SIZE))
  {
    return HAL_ERROR;
  }

  primask = UART1_DMATxEnterCritical();

  if (uart6_dma_tx_count >= UART6_DMA_TX_QUEUE_LENGTH)
  {
    UART1_DMATxExitCritical(primask);
    return HAL_BUSY;
  }

  queue_head = uart6_dma_tx_head;
  memcpy(uart6_dma_tx_queue[queue_head].data, data, length);
  uart6_dma_tx_queue[queue_head].length = length;
  uart6_dma_tx_head = (uint8_t)((queue_head + 1U) % UART6_DMA_TX_QUEUE_LENGTH);
  uart6_dma_tx_count++;

  UART1_DMATxExitCritical(primask);

  UART6_DMATxStartNext();

  return HAL_OK;
}

static void Debug_SendUart6Telemetry(void)
{
  static uint32_t telemetry_sequence = 0U;
  char tx_buffer[UART6_DMA_TX_BUFFER_SIZE];
  int gyro_x;
  int gyro_y;
  int gyro_z;
  int roll_deg;
  int pitch_deg;
  int roll_output;
  int pitch_output;
  int yaw_output;
  int effective_roll_rate;
  int effective_pitch_rate;
  int roll_angle_error;
  int pitch_angle_error;
  int roll_rate_error;
  int pitch_rate_error;
  int roll_angle_output;
  int pitch_angle_output;
  int roll_angle_p_output;
  int roll_angle_d_output;
  int pitch_angle_p_output;
  int pitch_angle_d_output;
  int roll_rate_p_output;
  int pitch_rate_p_output;
  int accel_norm;
  int accel_trust;
  uint32_t channel_1_compare;
  uint32_t channel_2_compare;
  uint32_t channel_3_compare;
  uint32_t channel_4_compare;
  float roll_output_value;
  float pitch_output_value;
  float yaw_output_value;
  float effective_roll_rate_value;
  float effective_pitch_rate_value;
  float roll_angle_error_value;
  float pitch_angle_error_value;
  float roll_angle_output_value;
  float pitch_angle_output_value;
  float roll_angle_p_output_value;
  float roll_angle_d_output_value;
  float pitch_angle_p_output_value;
  float pitch_angle_d_output_value;
  float roll_rate_error_value;
  float pitch_rate_error_value;
  float roll_rate_p_output_value;
  float pitch_rate_p_output_value;
  int length;

  motor_get_channels(&channel_1_compare,
                     &channel_2_compare,
                     &channel_3_compare,
                     &channel_4_compare);
  motor_get_control_outputs(&roll_output_value,
                            &pitch_output_value,
                            &yaw_output_value,
                            &effective_roll_rate_value,
                            &effective_pitch_rate_value);
  motor_get_roll_tuning_debug(&roll_angle_error_value,
                              &roll_angle_output_value,
                              &roll_rate_error_value,
                              &roll_rate_p_output_value,
                              NULL,
                              NULL);
  motor_get_pitch_tuning_debug(&pitch_angle_error_value,
                               &pitch_angle_output_value,
                               &pitch_rate_error_value,
                               &pitch_rate_p_output_value,
                               NULL,
                               NULL);
  motor_get_angle_pd_debug(&roll_angle_p_output_value,
                           &roll_angle_d_output_value,
                           &pitch_angle_p_output_value,
                           &pitch_angle_d_output_value);

  gyro_x = (int)(sensor_gyro_x_dps * 100.0f);
  gyro_y = (int)(sensor_gyro_y_dps * 100.0f);
  gyro_z = (int)(sensor_gyro_z_dps * 100.0f);
  roll_deg = (int)(sensor_roll_deg * 100.0f);
  pitch_deg = (int)(sensor_pitch_deg * 100.0f);
  roll_output = (int)(roll_output_value * 100.0f);
  pitch_output = (int)(pitch_output_value * 100.0f);
  yaw_output = (int)(yaw_output_value * 100.0f);
  effective_roll_rate = (int)(effective_roll_rate_value * 100.0f);
  effective_pitch_rate = (int)(effective_pitch_rate_value * 100.0f);
  roll_angle_error = (int)(roll_angle_error_value * 100.0f);
  pitch_angle_error = (int)(pitch_angle_error_value * 100.0f);
  roll_rate_error = (int)(roll_rate_error_value * 100.0f);
  pitch_rate_error = (int)(pitch_rate_error_value * 100.0f);
  roll_angle_output = (int)(roll_angle_output_value * 100.0f);
  pitch_angle_output = (int)(pitch_angle_output_value * 100.0f);
  roll_angle_p_output = (int)(roll_angle_p_output_value * 100.0f);
  roll_angle_d_output = (int)(roll_angle_d_output_value * 100.0f);
  pitch_angle_p_output = (int)(pitch_angle_p_output_value * 100.0f);
  pitch_angle_d_output = (int)(pitch_angle_d_output_value * 100.0f);
  roll_rate_p_output = (int)(roll_rate_p_output_value * 100.0f);
  pitch_rate_p_output = (int)(pitch_rate_p_output_value * 100.0f);
  accel_norm = (int)(sensor_accel_norm_g * 100.0f);
  accel_trust = (int)(sensor_accel_trust * 100.0f);

#if 0
  /* Legacy Bluetooth telemetry is temporarily disabled for pitch tuning. */
  length = snprintf(tx_buffer,
                    sizeof(tx_buffer),
                    "-----------\r\n%lu,%lu,%lu,%lu,%lu,%lu,%s%d.%02d,%s%d.%02d,%s%d.%02d,%s%d.%02d,%s%d.%02d,%s%d.%02d,%s%d.%02d,%s%d.%02d,%s%d.%02d,%s%d.%02d\r\n",
                    (unsigned long)HAL_GetTick(),
                    (unsigned long)motor_get_throttle(),
                    (unsigned long)channel_1_compare,
                    (unsigned long)channel_2_compare,
                    (unsigned long)channel_3_compare,
                    (unsigned long)channel_4_compare,
                    (gyro_x < 0) ? "-" : "", DebugAbs(gyro_x) / 100, DebugAbs(gyro_x) % 100,
                    (gyro_y < 0) ? "-" : "", DebugAbs(gyro_y) / 100, DebugAbs(gyro_y) % 100,
                    (gyro_z < 0) ? "-" : "", DebugAbs(gyro_z) / 100, DebugAbs(gyro_z) % 100,
                    (roll_deg < 0) ? "-" : "", DebugAbs(roll_deg) / 100, DebugAbs(roll_deg) % 100,
                    (pitch_deg < 0) ? "-" : "", DebugAbs(pitch_deg) / 100, DebugAbs(pitch_deg) % 100,
                    (roll_output < 0) ? "-" : "", DebugAbs(roll_output) / 100, DebugAbs(roll_output) % 100,
                    (pitch_output < 0) ? "-" : "", DebugAbs(pitch_output) / 100, DebugAbs(pitch_output) % 100,
                    (yaw_output < 0) ? "-" : "", DebugAbs(yaw_output) / 100, DebugAbs(yaw_output) % 100,
                    (effective_roll_rate < 0) ? "-" : "", DebugAbs(effective_roll_rate) / 100, DebugAbs(effective_roll_rate) % 100,
                    (effective_pitch_rate < 0) ? "-" : "", DebugAbs(effective_pitch_rate) / 100, DebugAbs(effective_pitch_rate) % 100);
#endif

  length = snprintf(tx_buffer,
                    sizeof(tx_buffer),
                    "T,%lu,%lu,%lu,%lu,%lu,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                    (unsigned long)telemetry_sequence++,
                    (unsigned long)motor_get_throttle(),
                    (unsigned long)channel_1_compare,
                    (unsigned long)channel_2_compare,
                    (unsigned long)channel_3_compare,
                    (unsigned long)channel_4_compare,
                    gyro_x,
                    gyro_y,
                    gyro_z,
                    roll_deg,
                    pitch_deg,
                    roll_output,
                    pitch_output,
                    yaw_output,
                    effective_roll_rate,
                    effective_pitch_rate,
                    roll_angle_error,
                    pitch_angle_error,
                    roll_rate_error,
                    pitch_rate_error,
                    roll_angle_output,
                    pitch_angle_output,
                    roll_angle_p_output,
                    roll_angle_d_output,
                    pitch_angle_p_output,
                    pitch_angle_d_output,
                    roll_rate_p_output,
                    pitch_rate_p_output,
                    accel_norm,
                    accel_trust);

  if (length > 0)
  {
    if (length >= (int)sizeof(tx_buffer))
    {
      length = (int)sizeof(tx_buffer) - 1;
    }
    (void)UART6_DMATxEnqueue((uint8_t *)tx_buffer, (uint16_t)length);
  }
}

static void Debug_SendUart6SensorTiming(void)
{
  static uint32_t timing_sequence = 0U;
  char tx_buffer[160];
  float latest_dt_us;
  float average_dt_us;
  float minimum_dt_us;
  float maximum_dt_us;
  float gyro_input_x_dps;
  float gyro_input_y_dps;
  uint32_t sample_count;
  uint32_t clamped_count;
  int length;

  sensor_get_timing_debug(&latest_dt_us,
                          &sample_count,
                          &average_dt_us,
                          &minimum_dt_us,
                          &maximum_dt_us,
                          &clamped_count,
                          &gyro_input_x_dps,
                          &gyro_input_y_dps);

  length = snprintf(tx_buffer,
                    sizeof(tx_buffer),
                    "D,%lu,%d,%lu,%d,%d,%d,%lu,%d,%d,%d,%d\r\n",
                    (unsigned long)timing_sequence++,
                    (int)latest_dt_us,
                    (unsigned long)sample_count,
                    (int)average_dt_us,
                    (int)minimum_dt_us,
                    (int)maximum_dt_us,
                    (unsigned long)clamped_count,
                    (int)(gyro_input_x_dps * 100.0f),
                    (int)(gyro_input_y_dps * 100.0f),
                    (int)(sensor_gyro_x_dps * 100.0f),
                    (int)(sensor_gyro_y_dps * 100.0f));

  if (length > 0)
  {
    if (length >= (int)sizeof(tx_buffer))
    {
      length = (int)sizeof(tx_buffer) - 1;
    }
    (void)UART6_DMATxEnqueue((uint8_t *)tx_buffer, (uint16_t)length);
  }
}

static void Debug_SendHighSpeedEventNotification(void)
{
  high_speed_log_event_info_t info;
  char tx_buffer[96];
  int length;

  if (high_speed_log_get_pending_notification(&info) == 0U)
  {
    return;
  }

  length = snprintf(tx_buffer,
                    sizeof(tx_buffer),
                    "E,%u,%lu,%lu,%c,%u\r\n",
                    info.event_id,
                    (unsigned long)info.trigger_tick_ms,
                    (unsigned long)info.trigger_control_sequence,
                    (char)info.trigger_axis,
                    info.trigger_reason);
  if ((length > 0) && (length < (int)sizeof(tx_buffer)) &&
      (UART6_DMATxEnqueue((uint8_t *)tx_buffer, (uint16_t)length) == HAL_OK))
  {
    high_speed_log_ack_notification(info.event_id);
  }
}

static void Debug_ProcessHighSpeedLogDump(void)
{
  high_speed_log_event_info_t info;
  high_speed_log_sample_t sample;
  char tx_buffer[UART6_DMA_TX_BUFFER_SIZE];
  uint8_t event_count = high_speed_log_get_event_count();
  uint16_t batch_count = 0U;
  uint16_t batch_index;
  uint16_t remaining_samples;
  uint32_t current_tick = HAL_GetTick();
  int written;
  int length = 0;

  if (high_speed_dump_phase == HIGH_SPEED_DUMP_IDLE)
  {
    high_speed_dump_event_index = 0U;
    high_speed_dump_sample_index = 0U;
    high_speed_dump_last_send_tick = current_tick - UART6_HIGH_SPEED_DUMP_INTERVAL_MS;
    high_speed_dump_phase = HIGH_SPEED_DUMP_BEGIN;
  }

  if ((uint32_t)(current_tick - high_speed_dump_last_send_tick) <
      UART6_HIGH_SPEED_DUMP_INTERVAL_MS)
  {
    return;
  }

  switch (high_speed_dump_phase)
  {
    case HIGH_SPEED_DUMP_BEGIN:
      length = snprintf(tx_buffer, sizeof(tx_buffer), "HF_DUMP_BEGIN,%u\r\n", event_count);
      break;

    case HIGH_SPEED_DUMP_EVENT_BEGIN:
      if (high_speed_log_get_event_info(high_speed_dump_event_index, &info) == 0U)
      {
        high_speed_dump_phase = HIGH_SPEED_DUMP_END;
        return;
      }
      length = snprintf(tx_buffer,
                        sizeof(tx_buffer),
                        "HF_BEGIN,%u,%lu,%lu,%c,%u,%u,%u,%d,%d,%d,%d\r\n",
                        info.event_id,
                        (unsigned long)info.trigger_tick_ms,
                        (unsigned long)info.trigger_control_sequence,
                        (char)info.trigger_axis,
                        info.trigger_reason,
                        info.trigger_sample_index,
                        info.sample_count,
                        info.peak_gyro_x_dps10,
                        info.peak_gyro_y_dps10,
                        info.peak_roll_error_dps10,
                        info.peak_pitch_error_dps10);
      break;

    case HIGH_SPEED_DUMP_SAMPLE:
      if ((high_speed_log_get_event_info(high_speed_dump_event_index, &info) == 0U) ||
          (high_speed_dump_sample_index >= info.sample_count))
      {
        high_speed_dump_phase = HIGH_SPEED_DUMP_EVENT_END;
        return;
      }

      remaining_samples = (uint16_t)(info.sample_count - high_speed_dump_sample_index);
      batch_count = (remaining_samples > UART6_HIGH_SPEED_DUMP_BATCH_SAMPLES) ?
                    UART6_HIGH_SPEED_DUMP_BATCH_SAMPLES : remaining_samples;
      length = snprintf(tx_buffer,
                        sizeof(tx_buffer),
                        "HB,%u,%u,%u",
                        (unsigned int)(high_speed_dump_event_index + 1U),
                        high_speed_dump_sample_index,
                        batch_count);
      for (batch_index = 0U; batch_index < batch_count; batch_index++)
      {
        if (high_speed_log_get_sample(high_speed_dump_event_index,
                                      (uint16_t)(high_speed_dump_sample_index + batch_index),
                                      &sample) == 0U)
        {
          return;
        }
        written = snprintf(&tx_buffer[length],
                           sizeof(tx_buffer) - (size_t)length,
                           ",%u,%d,%d,%d,%d,%d,%d,%d,%d",
                           sample.dt_us,
                           sample.roll_cdeg,
                           sample.pitch_cdeg,
                           sample.roll_target_rate_dps10,
                           sample.pitch_target_rate_dps10,
                           sample.gyro_x_dps10,
                           sample.gyro_y_dps10,
                           sample.roll_output_pwm10,
                           sample.pitch_output_pwm10);
        if ((written <= 0) || (written >= ((int)sizeof(tx_buffer) - length)))
        {
          return;
        }
        length += written;
      }
      written = snprintf(&tx_buffer[length], sizeof(tx_buffer) - (size_t)length, "\r\n");
      if ((written <= 0) || (written >= ((int)sizeof(tx_buffer) - length)))
      {
        return;
      }
      length += written;
      break;

    case HIGH_SPEED_DUMP_EVENT_END:
      length = snprintf(tx_buffer,
                        sizeof(tx_buffer),
                        "HF_END,%u\r\n",
                        (unsigned int)(high_speed_dump_event_index + 1U));
      break;

    case HIGH_SPEED_DUMP_END:
      length = snprintf(tx_buffer, sizeof(tx_buffer), "HF_DUMP_END,%u\r\n", event_count);
      break;

    default:
      high_speed_dump_phase = HIGH_SPEED_DUMP_IDLE;
      high_speed_log_finish_dump();
      return;
  }

  if ((length <= 0) || (length >= (int)sizeof(tx_buffer)) ||
      (UART6_DMATxEnqueue((uint8_t *)tx_buffer, (uint16_t)length) != HAL_OK))
  {
    return;
  }

  high_speed_dump_last_send_tick = current_tick;

  switch (high_speed_dump_phase)
  {
    case HIGH_SPEED_DUMP_BEGIN:
      high_speed_dump_phase = (event_count == 0U) ? HIGH_SPEED_DUMP_END : HIGH_SPEED_DUMP_EVENT_BEGIN;
      break;
    case HIGH_SPEED_DUMP_EVENT_BEGIN:
      high_speed_dump_sample_index = 0U;
      high_speed_dump_phase = HIGH_SPEED_DUMP_SAMPLE;
      break;
    case HIGH_SPEED_DUMP_SAMPLE:
      high_speed_dump_sample_index = (uint16_t)(high_speed_dump_sample_index + batch_count);
      break;
    case HIGH_SPEED_DUMP_EVENT_END:
      high_speed_dump_event_index++;
      high_speed_dump_phase = (high_speed_dump_event_index < event_count) ?
                              HIGH_SPEED_DUMP_EVENT_BEGIN : HIGH_SPEED_DUMP_END;
      break;
    case HIGH_SPEED_DUMP_END:
      high_speed_dump_phase = HIGH_SPEED_DUMP_IDLE;
      high_speed_log_finish_dump();
      break;
    default:
      break;
  }
}

static void Debug_SendUartSnapshot(void)
{
  int gx_tenths = DebugSignScaledTenths(sensor_gyro_x_dps);
  int gy_tenths = DebugSignScaledTenths(sensor_gyro_y_dps);
  int gz_tenths = DebugSignScaledTenths(sensor_gyro_z_dps);
  int roll_tenths = DebugSignScaledTenths(sensor_roll_deg);
  int pitch_tenths = DebugSignScaledTenths(sensor_pitch_deg);
  int yaw_tenths = DebugSignScaledTenths(sensor_yaw_deg);
  float display_voltage = battery_voltage + 0.05f;
  uint8_t v_int = (uint8_t)display_voltage;
  uint8_t v_dec = (uint8_t)((display_voltage - v_int) * 10.0f);

  (void)uart1_printf("GX %c%d.%01d  GY %c%d.%01d  GZ %c%d.%01d  "
                     "BAT %d.%dV %d%%  R %c%d.%01d  P %c%d.%01d  Y %c%d.%01d\r\n",
                     (gx_tenths < 0) ? '-' : ' ', DebugIntegerPartFromScaled(gx_tenths, 10), DebugAbs(gx_tenths) % 10,
                     (gy_tenths < 0) ? '-' : ' ', DebugIntegerPartFromScaled(gy_tenths, 10), DebugAbs(gy_tenths) % 10,
                     (gz_tenths < 0) ? '-' : ' ', DebugIntegerPartFromScaled(gz_tenths, 10), DebugAbs(gz_tenths) % 10,
                     v_int, v_dec, battery_percent,
                     (roll_tenths < 0) ? '-' : ' ', DebugIntegerPartFromScaled(roll_tenths, 10), DebugAbs(roll_tenths) % 10,
                     (pitch_tenths < 0) ? '-' : ' ', DebugIntegerPartFromScaled(pitch_tenths, 10), DebugAbs(pitch_tenths) % 10,
                     (yaw_tenths < 0) ? '-' : ' ', DebugIntegerPartFromScaled(yaw_tenths, 10), DebugAbs(yaw_tenths) % 10);
}

static void Debug_UpdateOledSnapshot(void)
{
  int gx_tenths = DebugSignScaledTenths(sensor_gyro_x_dps);
  int gy_tenths = DebugSignScaledTenths(sensor_gyro_y_dps);
  int gz_tenths = DebugSignScaledTenths(sensor_gyro_z_dps);
  int roll_tenths = DebugSignScaledTenths(sensor_roll_deg);
  int pitch_tenths = DebugSignScaledTenths(sensor_pitch_deg);
  int yaw_tenths = DebugSignScaledTenths(sensor_yaw_deg);
  char gx_sign = (gx_tenths < 0) ? '-' : ' ';
  char gy_sign = (gy_tenths < 0) ? '-' : ' ';
  char gz_sign = (gz_tenths < 0) ? '-' : ' ';
  char roll_sign = (roll_tenths < 0) ? '-' : ' ';
  char pitch_sign = (pitch_tenths < 0) ? '-' : ' ';
  char yaw_sign = (yaw_tenths < 0) ? '-' : ' ';

  float display_voltage = battery_voltage + 0.05f; // 소수점 첫째자리 반올림
  uint8_t v_int = (uint8_t)display_voltage;
  uint8_t v_dec = (uint8_t)((display_voltage - v_int) * 10.0f);

  OLED_Clear();
  OLED_Printf(0, 0, "GX %c%d.%01d", gx_sign, DebugIntegerPartFromScaled(gx_tenths, 10), DebugAbs(gx_tenths) % 10);
  OLED_Printf(1, 0, "GY %c%d.%01d", gy_sign, DebugIntegerPartFromScaled(gy_tenths, 10), DebugAbs(gy_tenths) % 10);
  OLED_Printf(2, 0, "GZ %c%d.%01d", gz_sign, DebugIntegerPartFromScaled(gz_tenths, 10), DebugAbs(gz_tenths) % 10);
  OLED_Printf(3, 0, "BAT %d.%dV %d%%", v_int, v_dec, battery_percent);
  OLED_Printf(4, 0, "                ");
  OLED_Printf(5, 0, "                ");
  OLED_Printf(6, 0, "R %c%d.%01d P %c%d.%01d", roll_sign, DebugIntegerPartFromScaled(roll_tenths, 10), DebugAbs(roll_tenths) % 10,
                                          pitch_sign, DebugIntegerPartFromScaled(pitch_tenths, 10), DebugAbs(pitch_tenths) % 10);
  OLED_Printf(7, 0, "Y %c%d.%01d", yaw_sign, DebugIntegerPartFromScaled(yaw_tenths, 10), DebugAbs(yaw_tenths) % 10);
  OLED_Update();
}

void debug_init(void)
{
  debug_uart_update_flag = 0U;
  debug_uart6_update_flag = 0U;
  debug_oled_update_flag = 0U;
  debug_oled_tick_divider = 0U;
  main_flag = 0U;
  uart1_rx_float_value = 0.0f;
  uart1_trigger_active = 0U;
  uart1_trigger_start_tick = 0U;
  uart6_rx_float_value = 0.0f;
  uart6_start_received = 0U;
  uart6_arm_requested = 0U;
  uart6_disarm_requested = 0U;
  debug_bridge_mode_active = 0U;
  high_speed_dump_phase = HIGH_SPEED_DUMP_IDLE;
  high_speed_dump_event_index = 0U;
  high_speed_dump_sample_index = 0U;
  high_speed_dump_last_send_tick = 0U;
  uart1_dma_tx_busy = 0U;
  uart1_dma_tx_head = 0U;
  uart1_dma_tx_tail = 0U;
  uart1_dma_tx_count = 0U;
  uart6_dma_tx_busy = 0U;
  uart6_dma_tx_head = 0U;
  uart6_dma_tx_tail = 0U;
  uart6_dma_tx_count = 0U;
  UART1_DMARxStart();
  UART6_DMARxStart();
}

void debug_set_bridge_mode(uint8_t active)
{
  uint32_t primask = UART1_DMATxEnterCritical();

  debug_bridge_mode_active = active;
  uart1_dma_tx_busy = 0U;
  uart1_dma_tx_head = 0U;
  uart1_dma_tx_tail = 0U;
  uart1_dma_tx_count = 0U;

  UART1_DMATxExitCritical(primask);

  (void)HAL_UART_AbortTransmit(&huart1);
  (void)HAL_UART_AbortReceive(&huart1);
  UART1_DMARxStart();
}

void debug_process(void)
{
  static const uint8_t arm_ok_text[] = "ARM_OK\r\n";
  static const uint8_t arm_denied_text[] = "ARM_DENIED\r\n";
  static const uint8_t disarm_ok_text[] = "DISARM_OK\r\n";

  if (uart6_disarm_requested != 0U)
  {
    uart6_disarm_requested = 0U;
    flight_control_disarm(FLIGHT_FAILSAFE_USER);
    (void)UART6_DMATxEnqueue(disarm_ok_text, (uint16_t)(sizeof(disarm_ok_text) - 1U));
  }

  if (uart6_arm_requested != 0U)
  {
    uart6_arm_requested = 0U;
    if (flight_control_request_arm(battery_voltage, battery_valid))
    {
      (void)UART6_DMATxEnqueue(arm_ok_text, (uint16_t)(sizeof(arm_ok_text) - 1U));
    }
    else
    {
      (void)UART6_DMATxEnqueue(arm_denied_text, (uint16_t)(sizeof(arm_denied_text) - 1U));
    }
  }

  if ((uart1_trigger_active != 0U) &&
      ((HAL_GetTick() - uart1_trigger_start_tick) >= UART1_TRIGGER_HOLD_MS))
  {
    uart1_trigger_active = 0U;
    uart1_rx_float_value = 0.0f;
    motor_set_rate_targets(0.0f, 0.0f, 0.0f);
    debug_oled_update_flag = 1U;
  }

  if (debug_uart_update_flag != 0U)
  {
    debug_uart_update_flag = 0U;
    if (debug_bridge_mode_active == 0U)
    {
      Debug_SendUartSnapshot();
    }
  }

  if (debug_oled_update_flag != 0U)
  {
    debug_oled_update_flag = 0U;
    // OLED_Clear();
    // OLED_Printf(3, 0, "MOTOR = %d", user_step_throttle_compare);
    // OLED_Update();
    Debug_UpdateOledSnapshot();
  }

  if (high_speed_log_dump_requested() != 0U)
  {
    debug_uart6_update_flag = 0U;
    Debug_ProcessHighSpeedLogDump();
    return;
  }

  Debug_SendHighSpeedEventNotification();

  if (debug_uart6_update_flag != 0U)
  {
    debug_uart6_update_flag = 0U;
    if (UART6_FULL_TELEMETRY_ENABLED != 0U)
    {
      Debug_SendUart6Telemetry();
    }
    else
    {
      Debug_SendUart6SensorTiming();
    }
  }
}

int uart1_printf(const char *format, ...)
{
  uint8_t tx_buffer[UART1_DMA_TX_BUFFER_SIZE];
  int length;
  va_list args;

  if (format == NULL)
  {
    return -1;
  }

  if (debug_bridge_mode_active != 0U)
  {
    return 0;
  }

  va_start(args, format);
  length = vsnprintf((char *)tx_buffer, sizeof(tx_buffer), format, args);
  va_end(args);

  if (length <= 0)
  {
    return length;
  }

  if (length >= (int)sizeof(tx_buffer))
  {
    length = (int)sizeof(tx_buffer) - 1;
  }

  if (UART1_DMATxEnqueue(tx_buffer, (uint16_t)length) != HAL_OK)
  {
    return -1;
  }

  return length;
}

HAL_StatusTypeDef debug_uart1_write_raw(const uint8_t *data, uint16_t length)
{
  if (debug_bridge_mode_active == 0U)
  {
    return HAL_BUSY;
  }

  return UART1_DMATxEnqueue(data, length);
}

HAL_StatusTypeDef debug_uart6_write_raw(const uint8_t *data, uint16_t length)
{
  return UART6_DMATxEnqueue(data, length);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uint32_t primask;

  if (huart == NULL)
  {
    return;
  }

  if (huart->Instance == USART6)
  {
    primask = UART1_DMATxEnterCritical();

    if (uart6_dma_tx_count > 0U)
    {
      uart6_dma_tx_tail = (uint8_t)((uart6_dma_tx_tail + 1U) % UART6_DMA_TX_QUEUE_LENGTH);
      uart6_dma_tx_count--;
    }

    uart6_dma_tx_busy = 0U;
    UART1_DMATxExitCritical(primask);

    UART6_DMATxStartNext();
    return;
  }

  if (huart->Instance != USART1)
  {
    return;
  }

  primask = UART1_DMATxEnterCritical();

  if (uart1_dma_tx_count > 0U)
  {
    uart1_dma_tx_tail = (uint8_t)((uart1_dma_tx_tail + 1U) % UART1_DMA_TX_QUEUE_LENGTH);
    uart1_dma_tx_count--;
  }

  uart1_dma_tx_busy = 0U;
  UART1_DMATxExitCritical(primask);

  UART1_DMATxStartNext();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uint32_t primask;

  if (huart == NULL)
  {
    return;
  }

  if (huart->Instance == USART2)
  {
    gnss_handle_uart_error(huart);
    return;
  }

  if (huart->Instance == USART6)
  {
    primask = UART1_DMATxEnterCritical();
    uart6_dma_tx_busy = 0U;
    UART1_DMATxExitCritical(primask);

    (void)HAL_UART_AbortTransmit(huart);
    (void)HAL_UART_AbortReceive(huart);
    UART6_DMARxStart();
    return;
  }

  if (huart->Instance != USART1)
  {
    return;
  }

  primask = UART1_DMATxEnterCritical();

  uart1_dma_tx_busy = 0U;
  UART1_DMATxExitCritical(primask);

  (void)HAL_UART_AbortTransmit(huart);

  UART1_DMARxStart();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == NULL)
  {
    return;
  }

  if (huart->Instance == USART6)
  {
    UART6_DMATxRespond(Size);
    UART6_DMARxStart();
    return;
  }

  if (huart->Instance != USART1)
  {
    return;
  }

  if (debug_bridge_mode_active != 0U)
  {
    (void)uart_bridge_enqueue_pc_data(uart1_dma_rx_buffer, Size);
    UART1_DMARxStart();
    return;
  }

  UART1_DMARxStoreFloat(Size);
  UART1_DMARxStart();
}
