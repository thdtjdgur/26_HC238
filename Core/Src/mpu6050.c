/**
  ******************************************************************************
  * @file           : mpu6050.c
  * @brief          : MPU6050 IMU 센서 드라이버 구현
  ******************************************************************************
  */

#include "mpu6050.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
  * @brief  MPU6050 초기화
  * @param  hi2c: I2C 핸들 포인터
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t check;
    uint8_t data;

    HAL_Delay(1000);  // 전원 안정화 대기

    // WHO_AM_I 확인
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, &check, 1, 1000) != HAL_OK)
    {
        printf("MPU6050 - I2C READ ERROR. ERROR CODE = 0x%lX\r\n", hi2c->ErrorCode);
        return HAL_ERROR;
    }

    printf("MPU6050 - ID = 0x%02X\r\n", check); //WHO_AM_I

    if (check != 0x72)
    {
        printf("MPU6050 - NOT FOUND.\r\n");
        return HAL_ERROR;
    }

    printf("MPU6050 - DETECTED.\r\n");

    // 1. Sleep 모드 해제 (PWR_MGMT_1 = 0x00)
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    HAL_Delay(10);  // Wake-up 대기

    // 2. Sample Rate = 1kHz / (1 + 7) = 125Hz
    data = 0x07;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_SMPLRT_DIV,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    // 3. DLPF 설정 (노이즈 감소)
    data = 0x03;  // DLPF_CFG = 3 (44Hz bandwidth)
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_CONFIG,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    // 4. 가속도계 설정: ±2g (FS_SEL = 0)
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_ACCEL_CONFIG,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    // 5. 자이로스코프 설정: ±250°/s (FS_SEL = 0)
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_GYRO_CONFIG,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    printf("MPU6050 - INITIALIZED.\r\n");

    // 6. I2C Bypass 모드 활성화 (GY-87: HMC5883L 직접 접근용)
    data = 0x02;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_INT_PIN_CFG,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    printf("MPU6050 - I2C BYPASS FOR HMC5883L IS ENABLED.\r\n");

    return HAL_OK;
}

/**
  * @brief  가속도 + 자이로 한번에 읽기
  * @param  hi2c: I2C 핸들 포인터
  * @param  data: 데이터 저장 구조체 포인터
  */
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data)
{
    uint8_t Rec_Data[14];

    // 0x3B부터 14바이트 연속 읽기 (가속도 6 + 온도 2 + 자이로 6)
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H,
                     I2C_MEMADD_SIZE_8BIT, Rec_Data, 14, 1000);

    // 가속도
    data->Accel_X_RAW = (int16_t)((Rec_Data[0] << 8) | Rec_Data[1]);
    data->Accel_Y_RAW = (int16_t)((Rec_Data[2] << 8) | Rec_Data[3]);
    data->Accel_Z_RAW = (int16_t)((Rec_Data[4] << 8) | Rec_Data[5]);

    // 자이로 (인덱스 8부터, 6-7은 온도)
    data->Gyro_X_RAW = (int16_t)((Rec_Data[8] << 8) | Rec_Data[9]);
    data->Gyro_Y_RAW = (int16_t)((Rec_Data[10] << 8) | Rec_Data[11]);
    data->Gyro_Z_RAW = (int16_t)((Rec_Data[12] << 8) | Rec_Data[13]);

    // 단위 변환
    data->Ax = (float)data->Accel_X_RAW / ACCEL_SCALE_2G;
    data->Ay = (float)data->Accel_Y_RAW / ACCEL_SCALE_2G;
    data->Az = (float)data->Accel_Z_RAW / ACCEL_SCALE_2G;

    data->Gx = (float)data->Gyro_X_RAW / GYRO_SCALE_250DPS;
    data->Gy = (float)data->Gyro_Y_RAW / GYRO_SCALE_250DPS;
    data->Gz = (float)data->Gyro_Z_RAW / GYRO_SCALE_250DPS;

    // 센서가 뒤집혀 실장됨 (Z축 반전)
    data->Az = -data->Az;
    data->Gz = -data->Gz;

    // Roll (Phi) = Atan2(Gy, Gz) - 가속도계 사용
    data->roll = atan2f(data->Ay, data->Az);

    // Gz2 = Gy * Sin(Phi) + Gz * Cos(Phi)
    float Gz2 = data->Ay * sinf(data->roll) + data->Az * cosf(data->roll);

    // Pitch (Theta) = Atan(-Gx / Gz2)
    data->pitch = atanf(-data->Ax / Gz2);

    // 도 단위 변환
    data->roll_deg = data->roll * (180.0f / M_PI);
    data->pitch_deg = data->pitch * (180.0f / M_PI);
}

/**
  * @brief  자이로 캘리브레이션 (정지 상태에서 호출)
  * @param  hi2c: I2C 핸들 포인터
  * @param  cal: 캘리브레이션 데이터 저장 구조체
  * @param  samples: 샘플 수 (추천: 500~1000)
  */
void MPU6050_Calibrate_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_Calibration_t *cal, uint16_t samples)
{
    MPU6050_Data_t temp;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;

    printf("MPU6050 - CALIBRATING GYRO... KEEP DEVICE STILL.\r\n");

    for (uint16_t i = 0; i < samples; i++)
    {
        MPU6050_Read_All(hi2c, &temp);
        sum_gx += temp.Gx;
        sum_gy += temp.Gy;
        sum_gz += temp.Gz;
        HAL_Delay(2);
    }

    cal->Gx_offset = sum_gx / samples;
    cal->Gy_offset = sum_gy / samples;
    cal->Gz_offset = sum_gz / samples;

    printf("MPU6050 - CALIBRATION COMPLETE.\r\n");
    printf("MPU6050 - OFFSETS : Gx=%.2f Gy=%.2f Gz=%.2f\r\n",
           cal->Gx_offset, cal->Gy_offset, cal->Gz_offset);
}

/**
  * @brief  캘리브레이션 적용된 데이터 읽기
  * @param  hi2c: I2C 핸들 포인터
  * @param  data: 데이터 저장 구조체 포인터
  * @param  cal: 캘리브레이션 데이터
  */
void MPU6050_Read_All_Calibrated(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data, MPU6050_Calibration_t *cal)
{
    MPU6050_Read_All(hi2c, data);

    // 오프셋 보정
    data->Gx -= cal->Gx_offset;
    data->Gy -= cal->Gy_offset;
    data->Gz -= cal->Gz_offset;
}
