/**
  ******************************************************************************
  * @file           : compass.c
  * @brief          : 나침반 시각화 구현
  ******************************************************************************
  */

#include "compass.h"
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
  * @brief 간단한 나침반 그리기
  */
void Compass_Draw(uint16_t cx, uint16_t cy, uint16_t radius, float heading)
{
    // 1. 이전 내용 지우기
    ST7789_DrawFilledCircle(cx, cy, radius - 1, BLACK);

    // 2. 외곽 원
    ST7789_DrawCircle(cx, cy, radius, WHITE);


    // 4. 바늘 계산
    float rad = heading * M_PI / 180.0f;
    int needle_len = radius;

    // 북쪽 바늘 (빨강)
    int nx = cx + (int)(needle_len * sinf(rad));
    int ny = cy - (int)(needle_len * cosf(rad));
    ST7789_DrawLine(cx, cy, nx, ny, YELLOW);


}

/**
  * @brief 나침반 + 숫자 표시
  */
void Compass_DrawWithValue(uint16_t cx, uint16_t cy, uint16_t radius, float heading)
{
    char buf[8];

    // 나침반 그리기
    Compass_Draw(cx, cy, radius, heading);

    // 각도 숫자 표시 (나침반 아래)
    snprintf(buf, sizeof(buf), "%3d", (int)heading);
    ST7789_WriteString(cx - 16, cy + radius + 5, buf, Font_11x18, WHITE, BLACK);
}
