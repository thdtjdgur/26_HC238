/**
  ******************************************************************************
  * @file           : mag_calib.h
  * @brief          : 지자기 센서 캘리브레이션 (Hard-iron + Soft-iron)
  ******************************************************************************
  */

#ifndef MAG_CALIB_H
#define MAG_CALIB_H

#include <stdint.h>

#define MAG_CALIB_NPARAM 10

/* 캘리브레이션 샘플 누적 구조체 */
typedef struct {
    double S[MAG_CALIB_NPARAM][MAG_CALIB_NPARAM];
    uint32_t n;
} MagCalibAccum_t;

/* 캘리브레이션 결과 구조체 */
typedef struct {
    double b[3];       // Hard-iron 오프셋
    double A[3][3];    // Soft-iron 보정 행렬
} MagCalib_t;

/* 함수 프로토타입 */
void MagCalib_Init(MagCalibAccum_t *acc);
void MagCalib_Push(MagCalibAccum_t *acc, float Mx, float My, float Mz);
int  MagCalib_Solve(const MagCalibAccum_t *acc, MagCalib_t *out);
void MagCalib_Apply(const MagCalib_t *cal, float Mx, float My, float Mz, float *out);

#endif /* MAG_CALIB_H */
