/**
  ******************************************************************************
  * @file           : lrf.c
  * @brief          : JC02-1 레이저 거리측정 모듈 드라이버 구현
  ******************************************************************************
  */

#include "lrf.h"
#include <stdio.h>
#include <string.h>

#define LRF_H1      0xAE
#define LRF_H2      0xA7
#define LRF_T1      0xBC
#define LRF_T2      0xBE

/* 파서 상태 */
enum { S_H1 = 0, S_H2, S_LEN, S_BODY, S_T1, S_T2 };

/* 제어 명령 (데이터시트 3.2 절) */
static const uint8_t CMD_RESET[]      = {0xAE, 0xA7, 0x05, 0x00, 0x0B, 0x00, 0x10, 0xBC, 0xBE};
static const uint8_t CMD_SINGLE[]     = {0xAE, 0xA7, 0x04, 0x00, 0x05, 0x09, 0xBC, 0xBE};
static const uint8_t CMD_CONT_START[] = {0xAE, 0xA7, 0x04, 0x00, 0x0E, 0x12, 0xBC, 0xBE};
static const uint8_t CMD_CONT_STOP[]  = {0xAE, 0xA7, 0x04, 0x00, 0x0F, 0x13, 0xBC, 0xBE};

static void lrf_send(LRF_Handle_t *h, const uint8_t *p, uint16_t n)
{
    if (h == NULL || h->huart == NULL) return;
    HAL_UART_Transmit(h->huart, (uint8_t *)p, n, 100);
}

/**
  * @brief  완성된 프레임 해석 (체크섬 검증 포함)
  */
static void lrf_frame(LRF_Handle_t *h)
{
    uint8_t n = (uint8_t)(h->len - 1);      /* body = ADDR + CMD + DATA... + CHK */
    if (n < 3) { h->bad_frames++; return; }

    uint8_t sum = h->len;
    for (uint8_t i = 0; i < (uint8_t)(n - 1); i++) {
        sum = (uint8_t)(sum + h->body[i]);
    }
    if (sum != h->body[n - 1]) { h->bad_frames++; return; }   /* 체크섬 불일치 */

    uint8_t        cmd = h->body[1];
    const uint8_t *d   = &h->body[2];
    uint8_t        dn  = (uint8_t)(n - 3);  /* 순수 DATA 길이 */

    h->rx_frames++;
    h->last_rx_tick = HAL_GetTick();

    if (cmd == 0x85 && dn >= 19) {
        /* 측정 응답 */
        int16_t dist = (int16_t)(((uint16_t)d[2] << 8) | d[3]);
        h->pitch_ddeg = (int16_t)(((uint16_t)d[0] << 8) | d[1]);
        h->unit       = d[18];
        if (dist > 0) {
            h->dist_dm = (uint16_t)dist;
            h->valid   = true;
        } else {
            h->valid   = false;
        }
        h->updated = true;
    }
    else if (cmd == 0x0E) {
        /* 측정 실패 응답 */
        h->valid   = false;
        h->updated = true;
    }
    /* 0x8B(리셋 ACK), 0x8E/0x8F(연속측정 ACK) 등은 무시 */
}

/**
  * @brief  바이트 단위 파서 (인터럽트 컨텍스트)
  */
static void lrf_parse(LRF_Handle_t *h, uint8_t b)
{
    /* 진단용 : 모든 수신 바이트를 기록 */
    h->rx_bytes++;
    h->raw[h->raw_head] = b;
    h->raw_head = (uint8_t)((h->raw_head + 1) % LRF_RAW_MAX);

    switch (h->state) {
    case S_H1:
        if (b == LRF_H1) h->state = S_H2;
        break;

    case S_H2:
        if (b == LRF_H2)      h->state = S_LEN;
        else if (b == LRF_H1) h->state = S_H2;
        else                  h->state = S_H1;
        break;

    case S_LEN:
        if (b >= 4 && b <= (LRF_BODY_MAX + 1)) {
            h->len   = b;
            h->idx   = 0;
            h->state = S_BODY;
        } else {
            h->bad_frames++;
            h->state = (b == LRF_H1) ? S_H2 : S_H1;
        }
        break;

    case S_BODY:
        h->body[h->idx++] = b;
        if (h->idx >= (uint8_t)(h->len - 1)) h->state = S_T1;
        break;

    case S_T1:
        if (b == LRF_T1) { h->state = S_T2; }
        else             { h->bad_frames++; h->state = S_H1; }
        break;

    case S_T2:
        if (b == LRF_T2) lrf_frame(h);
        else             h->bad_frames++;
        h->state = S_H1;
        break;

    default:
        h->state = S_H1;
        break;
    }
}

/* ========================================================================== */

void LRF_Init(LRF_Handle_t *h, UART_HandleTypeDef *huart,
              GPIO_TypeDef *en_port, uint16_t en_pin, bool power_on)
{
    if (h == NULL) return;

    memset(h, 0, sizeof(LRF_Handle_t));
    h->huart   = huart;
    h->en_port = en_port;
    h->en_pin  = en_pin;
    h->state   = S_H1;
    h->unit    = 1;

    /* 수신을 먼저 열어야 모듈의 "init ok" 응답까지 잡을 수 있다 */
    LRF_StartRx(h);
    LRF_Power(h, power_on);
    h->last_rx_tick = HAL_GetTick();
}

void LRF_Power(LRF_Handle_t *h, bool on)
{
    if (h == NULL || h->en_port == NULL) return;
    HAL_GPIO_WritePin(h->en_port, h->en_pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void LRF_StartRx(LRF_Handle_t *h)
{
    if (h == NULL || h->huart == NULL) return;
    h->state = S_H1;
    HAL_UART_Receive_IT(h->huart, &h->rx_byte, 1);
}

void LRF_SetBaud(LRF_Handle_t *h, uint32_t baud)
{
    if (h == NULL || h->huart == NULL) return;
    HAL_UART_Abort(h->huart);                 /* 진행 중 TX/RX 중단, 상태 READY */
    h->huart->Init.BaudRate = baud;
    HAL_UART_Init(h->huart);
    LRF_StartRx(h);
}

void LRF_Reset(LRF_Handle_t *h)            { lrf_send(h, CMD_RESET,      sizeof(CMD_RESET)); }
void LRF_MeasureSingle(LRF_Handle_t *h)    { lrf_send(h, CMD_SINGLE,     sizeof(CMD_SINGLE)); }
void LRF_ContinuousStart(LRF_Handle_t *h)  { lrf_send(h, CMD_CONT_START, sizeof(CMD_CONT_START)); }
void LRF_ContinuousStop(LRF_Handle_t *h)   { lrf_send(h, CMD_CONT_STOP,  sizeof(CMD_CONT_STOP)); }

void LRF_RxCpltCallback(LRF_Handle_t *h)
{
    if (h == NULL) return;
    lrf_parse(h, h->rx_byte);
    HAL_UART_Receive_IT(h->huart, &h->rx_byte, 1);
}

uint16_t LRF_GetDistance_dm(LRF_Handle_t *h)
{
    if (h == NULL) return 0;

    uint32_t dm = h->dist_dm;
    switch (h->unit) {
    case 2:  dm = dm * 9144u / 10000u; break;   /* yard -> m */
    case 3:  dm = dm * 3048u / 10000u; break;   /* feet  -> m */
    default: break;                             /* 1 = 이미 0.1m */
    }
    return (uint16_t)dm;
}

void LRF_RawToHex(LRF_Handle_t *h, char *out, size_t out_size, uint8_t n)
{
    if (h == NULL || out == NULL || out_size < 4) return;
    if (n > LRF_RAW_MAX) n = LRF_RAW_MAX;

    out[0] = '\0';
    size_t pos = 0;
    uint8_t head = h->raw_head;                 /* 다음에 쓸 위치 = 가장 오래된 것 */
    for (uint8_t i = 0; i < n; i++) {
        uint8_t idx = (uint8_t)((head + LRF_RAW_MAX - n + i) % LRF_RAW_MAX);
        int w = snprintf(&out[pos], out_size - pos, "%02X ", h->raw[idx]);
        if (w <= 0 || (size_t)w >= out_size - pos) break;
        pos += (size_t)w;
    }
}
