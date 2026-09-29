/**
  ******************************************************************************
  * @file           : hmc5883l.h
  * @brief          : HMC5883L 3축 지자기 센서 드라이버 헤더
  ******************************************************************************
  */

#ifndef HMC5883L_H
#define HMC5883L_H

#include "main.h"
#include "magcalib.h"

/* HMC5883L I2C 주소 */
#define HMC5883L_ADDR_7BIT       0x1E
#define HMC5883L_ADDR            (HMC5883L_ADDR_7BIT << 1)  // 0x3C

/* 레지스터 주소 */
#define HMC5883L_REG_CONFIG_A    0x00
#define HMC5883L_REG_CONFIG_B    0x01
#define HMC5883L_REG_MODE        0x02
#define HMC5883L_REG_DATA_X_MSB  0x03
#define HMC5883L_REG_ID_A        0x0A

/* 설정값 */
#define HMC5883L_CONFIG_A_DEFAULT  0x78  // 8샘플 평균, 75Hz
#define HMC5883L_CONFIG_B_DEFAULT  0xA0  // ±2.5 Ga, 660 LSB/Gauss
#define HMC5883L_MODE_CONTINUOUS   0x00

/* 스케일 팩터 */
#define MAG_SCALE_660              660.0f

/* 센서 데이터 구조체 */
typedef struct {
    int16_t Mag_X_RAW;
    int16_t Mag_Y_RAW;
    int16_t Mag_Z_RAW;
    float Mx, My, Mz;         // Gauss 단위 (raw)
    float Mx_cal, My_cal, Mz_cal;  // 캘리브레이션 적용됨
    float heading;            // 방위각 (도)
} HMC5883L_Data_t;

/* 함수 프로토타입 */
HAL_StatusTypeDef HMC5883L_Init(I2C_HandleTypeDef *hi2c);
void HMC5883L_Read_Raw(I2C_HandleTypeDef *hi2c, HMC5883L_Data_t *data);
void HMC5883L_Read_All(I2C_HandleTypeDef *hi2c, HMC5883L_Data_t *data,
                       float roll_rad, float pitch_rad, const MagCalib_t *cal);
int  HMC5883L_Calibrate(I2C_HandleTypeDef *hi2c, MagCalib_t *cal_out, uint16_t samples);

#endif /* HMC5883L_H */
