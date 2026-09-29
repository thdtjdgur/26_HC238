/**
  ******************************************************************************
  * @file           : gps_neo_m8n.h
  * @brief          : NEO-M8N GPS 모듈 드라이버 헤더
  ******************************************************************************
  */

#ifndef GPS_NEO_M8N_H
#define GPS_NEO_M8N_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* 버퍼 크기 정의 */
#define GPS_UART_BUFFER_SIZE    256
#define GPS_NMEA_MAX_LENGTH     82

/* GPS 데이터 구조체 */
typedef struct {
    double latitude;        // 위도 (십진수)
    double longitude;       // 경도 (십진수)
    float altitude;         // 고도 (m)
    float speed_knots;      // 속도 (knots)
    float course;           // 진행 방향 (도)
    uint8_t satellites;     // 위성 수
    uint8_t fix_quality;    // Fix 품질 (0=무효, 1=GPS, 2=DGPS)
    uint8_t hour;           // 시 (UTC)
    uint8_t minute;         // 분
    uint8_t second;         // 초
    uint8_t day;            // 일
    uint8_t month;          // 월
    uint16_t year;          // 년
    bool valid;             // 데이터 유효 여부
} GPS_Data_t;

/* GPS 핸들 구조체 */
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t rx_buffer[GPS_UART_BUFFER_SIZE];
    uint8_t nmea_buffer[GPS_NMEA_MAX_LENGTH];
    uint16_t rx_index;
    volatile bool line_received;
    GPS_Data_t data;
} GPS_Handle_t;

/* 함수 프로토타입 */
HAL_StatusTypeDef GPS_Init(GPS_Handle_t *gps, UART_HandleTypeDef *huart);
void GPS_UART_RxCallback(GPS_Handle_t *gps, uint8_t received_byte);
bool GPS_Process(GPS_Handle_t *gps);
bool GPS_GetPosition(GPS_Handle_t *gps, double *latitude, double *longitude);
bool GPS_IsValid(GPS_Handle_t *gps);

#endif /* GPS_NEO_M8N_H */
