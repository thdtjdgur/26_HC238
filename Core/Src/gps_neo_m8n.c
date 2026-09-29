/**
  ******************************************************************************
  * @file           : gps_neo_m8n.c
  * @brief          : NEO-M8N GPS 모듈 드라이버 구현
  ******************************************************************************
  */

#include "gps_neo_m8n.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* 내부 함수 프로토타입 */
static double nmea_to_decimal(const char *nmea_coord, char direction);
static bool parse_gga(GPS_Handle_t *gps, char *sentence);
static bool parse_rmc(GPS_Handle_t *gps, char *sentence);
static bool verify_checksum(const char *sentence);
static char *get_field(char *sentence, int field_num);

/**
  * @brief  GPS 모듈 초기화
  * @param  gps: GPS 핸들 포인터
  * @param  huart: UART 핸들 포인터
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef GPS_Init(GPS_Handle_t *gps, UART_HandleTypeDef *huart)
{
    if (gps == NULL || huart == NULL) {
        return HAL_ERROR;
    }

    gps->huart = huart;
    gps->rx_index = 0;
    gps->line_received = false;

    memset(&gps->data, 0, sizeof(GPS_Data_t));
    memset(gps->rx_buffer, 0, GPS_UART_BUFFER_SIZE);
    memset(gps->nmea_buffer, 0, GPS_NMEA_MAX_LENGTH);

    printf("GPS NEO-M8N - INITIALIZED.\r\n");

    return HAL_OK;
}

/**
  * @brief  UART 수신 콜백 (인터럽트에서 호출)
  * @param  gps: GPS 핸들 포인터
  * @param  received_byte: 수신된 바이트
  */
void GPS_UART_RxCallback(GPS_Handle_t *gps, uint8_t received_byte)
{
    if (gps == NULL) return;

    /* '$'로 시작하면 새 문장 시작 */
    if (received_byte == '$') {
        gps->rx_index = 0;
    }

    if (gps->rx_index < GPS_UART_BUFFER_SIZE - 1) {
        gps->rx_buffer[gps->rx_index++] = received_byte;

        /* 줄바꿈이면 문장 완료 */
        if (received_byte == '\n') {
            gps->rx_buffer[gps->rx_index] = '\0';
            memcpy(gps->nmea_buffer, gps->rx_buffer, gps->rx_index + 1);
            gps->line_received = true;
            gps->rx_index = 0;
        }
    } else {
        gps->rx_index = 0;
    }
}

/**
  * @brief  GPS 데이터 처리 (메인 루프에서 호출)
  * @param  gps: GPS 핸들 포인터
  * @retval true: 유효한 데이터 파싱됨, false: 그 외
  */
bool GPS_Process(GPS_Handle_t *gps)
{
    if (gps == NULL || !gps->line_received) {
        return false;
    }

    gps->line_received = false;
    char *sentence = (char *)gps->nmea_buffer;

    if (sentence[0] != '$') {
        return false;
    }

    if (!verify_checksum(sentence)) {
        return false;
    }

    /* GGA 문장 파싱 (위치, 고도, 위성 수) */
    if (strstr(sentence, "$GPGGA") || strstr(sentence, "$GNGGA")) {
        return parse_gga(gps, sentence);
    }
    /* RMC 문장 파싱 (위치, 속도, 날짜/시간) */
    else if (strstr(sentence, "$GPRMC") || strstr(sentence, "$GNRMC")) {
        return parse_rmc(gps, sentence);
    }

    return false;
}

/**
  * @brief  위치 데이터 획득
  * @param  gps: GPS 핸들 포인터
  * @param  latitude: 위도 저장 포인터
  * @param  longitude: 경도 저장 포인터
  * @retval true: 유효한 위치, false: 무효
  */
bool GPS_GetPosition(GPS_Handle_t *gps, double *latitude, double *longitude)
{
    if (gps == NULL || latitude == NULL || longitude == NULL) {
        return false;
    }

    if (!gps->data.valid) {
        return false;
    }

    *latitude = gps->data.latitude;
    *longitude = gps->data.longitude;

    return true;
}

/**
  * @brief  GPS 데이터 유효 여부 확인
  * @param  gps: GPS 핸들 포인터
  * @retval true: 유효, false: 무효
  */
bool GPS_IsValid(GPS_Handle_t *gps)
{
    if (gps == NULL) {
        return false;
    }
    return gps->data.valid;
}

/* ========== 내부 헬퍼 함수 ========== */

/**
  * @brief  NMEA 좌표를 십진수로 변환
  */
static double nmea_to_decimal(const char *nmea_coord, char direction)
{
    if (nmea_coord == NULL || strlen(nmea_coord) == 0) {
        return 0.0;
    }

    double raw = atof(nmea_coord);
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);
    double decimal = degrees + (minutes / 60.0);

    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }

    return decimal;
}

/**
  * @brief  NMEA 문장에서 특정 필드 추출
  *         결과를 caller가 제공한 버퍼에 복사
  * @param  sentence: NMEA 문장
  * @param  field_num: 필드 번호 (0부터)
  * @param  out: 결과 저장 버퍼
  * @param  out_size: 버퍼 크기
  */
static void get_field_safe(char *sentence, int field_num, char *out, int out_size)
{
    int current_field = 0;
    int field_start = 0;
    int i = 0;

    memset(out, 0, out_size);

    while (sentence[i] != '\0') {
        if (sentence[i] == ',' || sentence[i] == '*') {
            if (current_field == field_num) {
                int len = i - field_start;
                if (len > 0 && len < out_size) {
                    strncpy(out, &sentence[field_start], len);
                    out[len] = '\0';
                }
                return;
            }
            current_field++;
            field_start = i + 1;
        }
        i++;
    }
}

/**
  * @brief  NMEA 체크섬 검증
  */
static bool verify_checksum(const char *sentence)
{
    if (sentence[0] != '$') {
        return false;
    }

    uint8_t checksum = 0;
    int i = 1;

    while (sentence[i] != '*' && sentence[i] != '\0') {
        checksum ^= sentence[i];
        i++;
    }

    if (sentence[i] != '*') {
        return false;
    }

    char checksum_str[3];
    checksum_str[0] = sentence[i + 1];
    checksum_str[1] = sentence[i + 2];
    checksum_str[2] = '\0';

    uint8_t received_checksum = (uint8_t)strtol(checksum_str, NULL, 16);

    return (checksum == received_checksum);
}

/**
  * @brief  GGA 문장 파싱
  */
static bool parse_gga(GPS_Handle_t *gps, char *sentence)
{
    char time_str[20], lat_str[20], lat_dir[4], lon_str[20], lon_dir[4];
    char fix_str[8], sat_str[8], alt_str[16];

    get_field_safe(sentence, 1, time_str, sizeof(time_str));
    get_field_safe(sentence, 2, lat_str,  sizeof(lat_str));
    get_field_safe(sentence, 3, lat_dir,  sizeof(lat_dir));
    get_field_safe(sentence, 4, lon_str,  sizeof(lon_str));
    get_field_safe(sentence, 5, lon_dir,  sizeof(lon_dir));
    get_field_safe(sentence, 6, fix_str,  sizeof(fix_str));
    get_field_safe(sentence, 7, sat_str,  sizeof(sat_str));
    get_field_safe(sentence, 9, alt_str,  sizeof(alt_str));

    int fix = atoi(fix_str);
    if (fix == 0) {
        gps->data.valid = false;
        return false;
    }

    gps->data.fix_quality = fix;
    gps->data.satellites = atoi(sat_str);

    if (strlen(lat_str) > 0 && strlen(lon_str) > 0) {
        gps->data.latitude = nmea_to_decimal(lat_str, lat_dir[0]);
        gps->data.longitude = nmea_to_decimal(lon_str, lon_dir[0]);
        gps->data.valid = true;
    }

    if (strlen(alt_str) > 0) {
        gps->data.altitude = atof(alt_str);
    }

    if (strlen(time_str) >= 6) {
        gps->data.hour = (time_str[0] - '0') * 10 + (time_str[1] - '0');
        gps->data.minute = (time_str[2] - '0') * 10 + (time_str[3] - '0');
        gps->data.second = (time_str[4] - '0') * 10 + (time_str[5] - '0');
    }

    return gps->data.valid;
}

/**
  * @brief  RMC 문장 파싱
  */
static bool parse_rmc(GPS_Handle_t *gps, char *sentence)
{
    char time_str[20], status[4], lat_str[20], lat_dir[4];
    char lon_str[20], lon_dir[4], speed_str[16], course_str[16], date_str[12];

    get_field_safe(sentence, 1, time_str,   sizeof(time_str));
    get_field_safe(sentence, 2, status,     sizeof(status));
    get_field_safe(sentence, 3, lat_str,    sizeof(lat_str));
    get_field_safe(sentence, 4, lat_dir,    sizeof(lat_dir));
    get_field_safe(sentence, 5, lon_str,    sizeof(lon_str));
    get_field_safe(sentence, 6, lon_dir,    sizeof(lon_dir));
    get_field_safe(sentence, 7, speed_str,  sizeof(speed_str));
    get_field_safe(sentence, 8, course_str, sizeof(course_str));
    get_field_safe(sentence, 9, date_str,   sizeof(date_str));

    if (status[0] != 'A') {
        gps->data.valid = false;
        return false;
    }

    if (strlen(lat_str) > 0 && strlen(lon_str) > 0) {
        gps->data.latitude = nmea_to_decimal(lat_str, lat_dir[0]);
        gps->data.longitude = nmea_to_decimal(lon_str, lon_dir[0]);
        gps->data.valid = true;
    }

    if (strlen(speed_str) > 0) {
        gps->data.speed_knots = atof(speed_str);
    }

    if (strlen(course_str) > 0) {
        gps->data.course = atof(course_str);
    }

    if (strlen(time_str) >= 6) {
        gps->data.hour = (time_str[0] - '0') * 10 + (time_str[1] - '0');
        gps->data.minute = (time_str[2] - '0') * 10 + (time_str[3] - '0');
        gps->data.second = (time_str[4] - '0') * 10 + (time_str[5] - '0');
    }

    if (strlen(date_str) >= 6) {
        gps->data.day = (date_str[0] - '0') * 10 + (date_str[1] - '0');
        gps->data.month = (date_str[2] - '0') * 10 + (date_str[3] - '0');
        gps->data.year = 2000 + (date_str[4] - '0') * 10 + (date_str[5] - '0');
    }

    return gps->data.valid;
}
