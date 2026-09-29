/**
  ******************************************************************************
  * @file           : compass.h
  * @brief          : 나침반 시각화 헤더
  ******************************************************************************
  */

#ifndef COMPASS_H
#define COMPASS_H

#include "st7789.h"
#include <stdint.h>

/**
  * @brief 간단한 나침반 그리기
  * @param cx: 중심 X 좌표
  * @param cy: 중심 Y 좌표
  * @param radius: 반지름
  * @param heading: 헤딩 각도 (0~360, 0=북쪽)
  */
void Compass_Draw(uint16_t cx, uint16_t cy, uint16_t radius, float heading);

/**
  * @brief 나침반 + 숫자 표시
  * @param cx: 중심 X 좌표
  * @param cy: 중심 Y 좌표
  * @param radius: 반지름
  * @param heading: 헤딩 각도 (0~360, 0=북쪽)
  */
void Compass_DrawWithValue(uint16_t cx, uint16_t cy, uint16_t radius, float heading);

#endif /* COMPASS_H */
