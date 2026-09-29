/**
  ******************************************************************************
  * @file           : lrf.h
  * @brief          : JC02-1 레이저 거리측정 모듈 드라이버 (UART, TTL)
  ******************************************************************************
  *
  *  [프레임 구조]  AE A7 LEN ADDR CMD DATA... CHK BC BE
  *     LEN  : LEN 바이트부터 CHK 바이트까지의 길이 (LEN+ADDR+CMD+DATA+CHK)
  *     ADDR : 모듈 주소 (공장 출하 0x00)
  *     CHK  : (LEN + ADDR + CMD + DATA) & 0xFF
  *
  *  [측정 응답]  CMD = 0x85, DATA 19바이트 (모두 big-endian signed short)
  *     [0:1]  부각(0.1°)      [2:3]  직선거리      [4:5]   정현고
  *     [6:7]  수평거리        [8:9]  두 점 높이    [10:11] 방위각
  *     [12:13] 수평 협각      [14:15] 잔여거리     [16:17] 속도(0.1km/h)
  *     [18]   거리 단위  1 = 0.1m / 2 = 0.1yd / 3 = 0.1ft
  *     (본 모듈은 '직선거리' 1개 항목만 실제로 지원)
  *
  *  [측정 실패 응답]  AE A7 04 00 0E 12 BC BE  (CMD = 0x0E)
  ******************************************************************************
  */

#ifndef LRF_H
#define LRF_H

#include "main.h"
#include <stdbool.h>

#define LRF_BODY_MAX    32
#define LRF_RAW_MAX     16      /* 진단용 최근 수신 바이트 링버퍼 */

typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef       *en_port;      /* EN(Power ON) 핀 - High 활성 */
    uint16_t            en_pin;

    /* --- 수신 파서 --- */
    uint8_t  rx_byte;                 /* HAL_UART_Receive_IT 용 1바이트 */
    uint8_t  state;
    uint8_t  len;
    uint8_t  idx;
    uint8_t  body[LRF_BODY_MAX];

    /* --- 측정 결과 --- */
    volatile bool     updated;        /* 새 응답 도착 (읽고 나면 false 로) */
    volatile bool     valid;          /* 마지막 측정 성공 여부 */
    volatile uint16_t dist_dm;        /* 직선거리 [0.1 m 단위] */
    volatile int16_t  pitch_ddeg;     /* 부각 [0.1 deg] */
    volatile uint8_t  unit;           /* 원본 거리 단위 코드 */

    /* --- 진단 카운터 --- */
    volatile uint32_t rx_bytes;       /* 수신한 전체 바이트 수 */
    volatile uint32_t rx_frames;      /* 체크섬까지 통과한 프레임 수 */
    volatile uint32_t bad_frames;     /* 헤더는 맞았지만 깨진 프레임 수 */
    volatile uint32_t last_rx_tick;   /* 마지막 프레임 수신 시각 */
    volatile uint8_t  raw[LRF_RAW_MAX];   /* 최근 수신 바이트 (링) */
    volatile uint8_t  raw_head;
} LRF_Handle_t;

/* power_on = false 로 두면 EN 을 Low 로 유지 (레이저 미동작, 통신만 준비) */
void LRF_Init(LRF_Handle_t *h, UART_HandleTypeDef *huart,
              GPIO_TypeDef *en_port, uint16_t en_pin, bool power_on);
void LRF_Power(LRF_Handle_t *h, bool on);
void LRF_StartRx(LRF_Handle_t *h);
void LRF_SetBaud(LRF_Handle_t *h, uint32_t baud);
void LRF_Reset(LRF_Handle_t *h);
void LRF_MeasureSingle(LRF_Handle_t *h);
void LRF_ContinuousStart(LRF_Handle_t *h);
void LRF_ContinuousStop(LRF_Handle_t *h);

/* USART2 수신 완료 인터럽트에서 호출 */
void LRF_RxCpltCallback(LRF_Handle_t *h);

/* 원본 단위 -> 미터(0.1m 단위)로 환산 */
uint16_t LRF_GetDistance_dm(LRF_Handle_t *h);

/* 최근 수신 바이트를 "AE A7 17 .." 형태의 hex 문자열로 (n = 최대 LRF_RAW_MAX) */
void LRF_RawToHex(LRF_Handle_t *h, char *out, size_t out_size, uint8_t n);

#endif /* LRF_H */
