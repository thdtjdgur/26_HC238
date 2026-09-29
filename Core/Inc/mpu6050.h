/**
  ******************************************************************************
  * @file           : mpu6050.h
  * @brief          : MPU6050 IMU 센서 드라이버 헤더
  ******************************************************************************
  */

#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"

/* MPU6050 I2C 주소 */
#define MPU6050_ADDR_7BIT    0x68
#define MPU6050_ADDR         (MPU6050_ADDR_7BIT << 1)  // 0xD0

/* 레지스터 주소 */
#define MPU6050_WHO_AM_I         0x75
#define MPU6050_PWR_MGMT_1       0x6B
#define MPU6050_SMPLRT_DIV       0x19
#define MPU6050_CONFIG           0x1A
#define MPU6050_GYRO_CONFIG      0x1B
#define MPU6050_ACCEL_CONFIG     0x1C
#define MPU6050_ACCEL_XOUT_H     0x3B
#define MPU6050_GYRO_XOUT_H      0x43
#define MPU6050_INT_PIN_CFG      0x37  // I2C Bypass 설정용

/* 스케일 팩터 */
#define ACCEL_SCALE_2G       16384.0f
#define GYRO_SCALE_250DPS    131.0f

/* 센서 데이터 구조체 */
typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    float Ax, Ay, Az;    // g 단위
    float Gx, Gy, Gz;    // deg/s 단위
    float roll;          // Roll (Phi) - 라디안
    float pitch;         // Pitch (Theta) - 라디안
    float roll_deg;      // Roll - 도
    float pitch_deg;     // Pitch - 도
} MPU6050_Data_t;

/* 캘리브레이션 오프셋 구조체 */
typedef struct {
    float Gx_offset;
    float Gy_offset;
    float Gz_offset;
} MPU6050_Calibration_t;

/* 함수 프로토타입 */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data);
void MPU6050_Calibrate_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_Calibration_t *cal, uint16_t samples);
void MPU6050_Read_All_Calibrated(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data, MPU6050_Calibration_t *cal);

#endif /* MPU6050_H */
