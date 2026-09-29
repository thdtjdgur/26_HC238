/**
  ******************************************************************************
  * @file           : hmc5883l.c
  * @brief          : HMC5883L 3축 지자기 센서 드라이버 구현
  ******************************************************************************
  */

#include "hmc5883l.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
  * @brief  HMC5883L 초기화
  */
HAL_StatusTypeDef HMC5883L_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t check[3];
    uint8_t data;

    HAL_Delay(1000);

    if (HAL_I2C_Mem_Read(hi2c, HMC5883L_ADDR, HMC5883L_REG_ID_A,
                         I2C_MEMADD_SIZE_8BIT, check, 3, 1000) != HAL_OK)
    {
        printf("HMC5883L - I2C READ ERROR. ERROR CODE = 0x%lX\r\n", hi2c->ErrorCode);
        return HAL_ERROR;
    }

    printf("HMC5883L - ID = %c%c%c\r\n", check[0], check[1], check[2]);

    if (check[0] != 'H' || check[1] != '4' || check[2] != '3')
    {
        printf("HMC5883L - NOT FOUND.\r\n");
        return HAL_ERROR;
    }

    printf("HMC5883L - DETECTED.\r\n");

    data = HMC5883L_CONFIG_A_DEFAULT;
    HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, HMC5883L_REG_CONFIG_A,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    data = HMC5883L_CONFIG_B_DEFAULT;
    HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, HMC5883L_REG_CONFIG_B,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    data = HMC5883L_MODE_CONTINUOUS;
    HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, HMC5883L_REG_MODE,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    HAL_Delay(10);

    printf("HMC5883L - INITIALIZED.\r\n");
    return HAL_OK;
}

/**
  * @brief  Raw 데이터만 읽기 (캘리브레이션 샘플 수집용)
  */
void HMC5883L_Read_Raw(I2C_HandleTypeDef *hi2c, HMC5883L_Data_t *data)
{
    uint8_t Rec_Data[6];

    HAL_I2C_Mem_Read(hi2c, HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB,
                     I2C_MEMADD_SIZE_8BIT, Rec_Data, 6, 1000);

    data->Mag_X_RAW = (int16_t)((Rec_Data[0] << 8) | Rec_Data[1]);
    data->Mag_Z_RAW = (int16_t)((Rec_Data[2] << 8) | Rec_Data[3]);
    data->Mag_Y_RAW = (int16_t)((Rec_Data[4] << 8) | Rec_Data[5]);

    data->Mx = (float)data->Mag_X_RAW / MAG_SCALE_660;
    data->My = (float)data->Mag_Y_RAW / MAG_SCALE_660;
    data->Mz = (float)data->Mag_Z_RAW / MAG_SCALE_660;

    // 센서가 뒤집혀 실장됨
    data->Mz = -data->Mz;
}

/**
  * @brief  캘리브레이션 + Tilt 보정된 Heading 읽기
  * @param  cal: 캘리브레이션 데이터 (NULL이면 캘리브레이션 없이 계산)
  */
void HMC5883L_Read_All(I2C_HandleTypeDef *hi2c, HMC5883L_Data_t *data,
                       float roll_rad, float pitch_rad, const MagCalib_t *cal)
{
    // Raw 데이터 읽기
    HMC5883L_Read_Raw(hi2c, data);

    float Bx, By, Bz;

    // 캘리브레이션 적용
    if (cal != NULL)
    {
        float cal_out[3];
        MagCalib_Apply(cal, data->Mx, data->My, data->Mz, cal_out);
        Bx = cal_out[0];
        By = cal_out[1];
        Bz = cal_out[2];
        data->Mx_cal = Bx;
        data->My_cal = By;
        data->Mz_cal = Bz;
    }
    else
    {
        Bx = data->Mx;
        By = data->My;
        Bz = data->Mz;
        data->Mx_cal = Bx;
        data->My_cal = By;
        data->Mz_cal = Bz;
    }

    // Tilt compensation
    float sin_phi = sinf(roll_rad);
    float cos_phi = cosf(roll_rad);
    float sin_theta = sinf(pitch_rad);
    float cos_theta = cosf(pitch_rad);

    // By2 = Bz * sin(Phi) - By * cos(Phi)
    float By2 = Bz * sin_phi - By * cos_phi;

    // Bz2 = By * sin(Phi) + Bz * cos(Phi)
    float Bz2 = By * sin_phi + Bz * cos_phi;

    // Bx3 = Bx * cos(Theta) + Bz2 * sin(Theta)
    float Bx3 = Bx * cos_theta + Bz2 * sin_theta;

    // Yaw = atan2(By2, Bx3)
    float heading_rad = atan2f(By2, Bx3);
    float heading_deg = heading_rad * (180.0f / M_PI);

    if (heading_deg < 0)
    {
        heading_deg += 360.0f;
    }

    data->heading = heading_deg;
}



/**
  * @brief  지자기 센서 캘리브레이션 수행
  * @param  hi2c: I2C 핸들 포인터
  * @param  cal_out: 캘리브레이션 결과 저장
  * @param  samples: 샘플 수 (추천: 500~1000)
  * @retval 0: 성공, 음수: 실패
  */
int HMC5883L_Calibrate(I2C_HandleTypeDef *hi2c, MagCalib_t *cal_out, uint16_t samples)
{
    MagCalibAccum_t accum;
    HMC5883L_Data_t data;

    printf("=== MAGNETOMETER CALIBRATION ===\r\n");
    printf("Rotate sensor in all directions (figure-8 motion)\r\n");
    printf("Collecting %d samples...\r\n\r\n", samples);

    MagCalib_Init(&accum);

    for (uint16_t i = 0; i < samples; i++)
    {
        HMC5883L_Read_Raw(hi2c, &data);
        MagCalib_Push(&accum, data.Mx, data.My, data.Mz);

        if (i % 100 == 0)
        {
            printf("Sample %d/%d\r\n", i, samples);
        }
        HAL_Delay(20);
    }

    printf("\r\nSolving calibration...\r\n");

    int result = MagCalib_Solve(&accum, cal_out);

    if (result == 0)
    {
        printf("\r\n=== CALIBRATION SUCCESS ===\r\n");
        printf("Copy this to your code:\r\n\r\n");
        printf("MagCalib_t mag_cal = {\r\n");
        printf("    .b = {%.6f, %.6f, %.6f},\r\n",
               cal_out->b[0], cal_out->b[1], cal_out->b[2]);
        printf("    .A = {\r\n");
        printf("        {%.6f, %.6f, %.6f},\r\n",
               cal_out->A[0][0], cal_out->A[0][1], cal_out->A[0][2]);
        printf("        {%.6f, %.6f, %.6f},\r\n",
               cal_out->A[1][0], cal_out->A[1][1], cal_out->A[1][2]);
        printf("        {%.6f, %.6f, %.6f}\r\n",
               cal_out->A[2][0], cal_out->A[2][1], cal_out->A[2][2]);
        printf("    }\r\n");
        printf("};\r\n");
    }
    else
    {
        printf("Calibration FAILED! Error code: %d\r\n", result);
        printf("Try collecting more samples or rotating more evenly.\r\n");
    }

    return result;
}


