/**
  ******************************************************************************
  * @file           : st7789.h
  * @brief          : ST7789 LCD 드라이버 헤더 (240x240, SPI + DMA)
  ******************************************************************************
  */

#ifndef __ST7789_H
#define __ST7789_H

#include "fonts.h"
#include "main.h"

/*******************************************************************************
 * SPI 포트 설정
 ******************************************************************************/
#define ST7789_SPI_PORT hspi1
extern SPI_HandleTypeDef ST7789_SPI_PORT;

/*******************************************************************************
 * DMA 사용 여부 (주석 처리하면 폴링 모드 사용)
 ******************************************************************************/
#define USE_DMA

/*******************************************************************************
 * CS 핀 사용 여부 (CS가 GND에 연결된 경우 아래 주석 해제)
 ******************************************************************************/
// #define CFG_NO_CS

/*******************************************************************************
 * 핀 연결 설정 (main.h에서 정의됨)
 ******************************************************************************/
#define ST7789_RST_PORT ST7789_RST_GPIO_Port
#define ST7789_RST_PIN  ST7789_RST_Pin
#define ST7789_DC_PORT  ST7789_DC_GPIO_Port
#define ST7789_DC_PIN   ST7789_DC_Pin

#ifndef CFG_NO_CS
#define ST7789_CS_PORT  ST7789_CS_GPIO_Port
#define ST7789_CS_PIN   ST7789_CS_Pin
#endif

/* 백라이트 제어가 필요한 경우 (main.h에서 정의 필요) */
// #define BLK_PORT ST7789_BLK_GPIO_Port
// #define BLK_PIN  ST7789_BLK_Pin

/*******************************************************************************
 * 디스플레이 해상도 설정
 * 사용하는 LCD에 맞게 하나만 정의
 ******************************************************************************/
// #define USING_135X240    /* 0.96인치 */
#define USING_240X240       /* 1.3인치 (기본) */
// #define USING_170X320    /* 1.9인치 */

/*******************************************************************************
 * 디스플레이 회전 설정 (0-3)
 ******************************************************************************/
// #define ST7789_ROTATION 0   /* 세로 (180도) */
#define ST7789_ROTATION 3      /* 2번 기준 반시계 90도  <-- 현재 설정 */
// #define ST7789_ROTATION 2   /* 세로 (기본) */
// #define ST7789_ROTATION 3   /* 2번 기준 시계 90도 */
/* 화면이 원하는 방향의 반대(시계방향)로 돌아가 있으면 1 -> 3 으로만 바꾸면 된다.
   240x240 은 회전에 따라 X_SHIFT/Y_SHIFT 가 자동으로 잡히므로 그 외 수정은 불필요. */

/*******************************************************************************
 * 좌우 반전 (거울 모드)
 *   1 = 화면을 좌우로 뒤집는다. 프리즘/빔스플리터로 반사시켜 보는 광학계에서 필요.
 *       위의 ST7789_ROTATION 회전에 이어서 적용된다.
 *   0 = 반전 없음
 *
 * 반전 축이 반대(상하로 뒤집힘)라면 ST7789_ROTATION 을 1 <-> 3 으로 바꾸면 된다.
 ******************************************************************************/
#define ST7789_MIRROR_X 1

/*******************************************************************************
 * 해상도별 설정값 (자동 설정됨)
 ******************************************************************************/
#ifdef USING_135X240
    #if ST7789_ROTATION == 0
        #define ST7789_WIDTH 135
        #define ST7789_HEIGHT 240
        #define X_SHIFT 53
        #define Y_SHIFT 40
    #elif ST7789_ROTATION == 1
        #define ST7789_WIDTH 240
        #define ST7789_HEIGHT 135
        #define X_SHIFT 40
        #define Y_SHIFT 52
    #elif ST7789_ROTATION == 2
        #define ST7789_WIDTH 135
        #define ST7789_HEIGHT 240
        #define X_SHIFT 52
        #define Y_SHIFT 40
    #elif ST7789_ROTATION == 3
        #define ST7789_WIDTH 240
        #define ST7789_HEIGHT 135
        #define X_SHIFT 40
        #define Y_SHIFT 53
    #endif
#endif

#ifdef USING_240X240
    #define ST7789_WIDTH 240
    #define ST7789_HEIGHT 240
    #if ST7789_ROTATION == 0
        #define X_SHIFT 0
        #define Y_SHIFT 80
    #elif ST7789_ROTATION == 1
        /* 좌우 반전 시 MADCTL 의 MY 비트가 빠지므로 320축 오프셋도 사라진다 */
        #if ST7789_MIRROR_X
            #define X_SHIFT 0
        #else
            #define X_SHIFT 80
        #endif
        #define Y_SHIFT 0
    #elif ST7789_ROTATION == 2
        #define X_SHIFT 0
        #define Y_SHIFT 0
    #elif ST7789_ROTATION == 3
        /* 좌우 반전 시 MY 비트가 추가되므로 320축 오프셋이 생긴다 */
        #if ST7789_MIRROR_X
            #define X_SHIFT 80
        #else
            #define X_SHIFT 0
        #endif
        #define Y_SHIFT 0
    #endif
#endif

#ifdef USING_170X320
    #if ST7789_ROTATION == 0
        #define ST7789_WIDTH 170
        #define ST7789_HEIGHT 320
        #define X_SHIFT 35
        #define Y_SHIFT 0
    #elif ST7789_ROTATION == 1
        #define ST7789_WIDTH 320
        #define ST7789_HEIGHT 170
        #define X_SHIFT 0
        #define Y_SHIFT 35
    #elif ST7789_ROTATION == 2
        #define ST7789_WIDTH 170
        #define ST7789_HEIGHT 320
        #define X_SHIFT 35
        #define Y_SHIFT 0
    #elif ST7789_ROTATION == 3
        #define ST7789_WIDTH 320
        #define ST7789_HEIGHT 170
        #define X_SHIFT 0
        #define Y_SHIFT 35
    #endif
#endif

/*******************************************************************************
 * 색상 정의 (RGB565 형식)
 ******************************************************************************/
#define WHITE       0xFFFF
#define BLACK       0x0000
#define BLUE        0x001F
#define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define GRAY        0X8430
#define BRED        0XF81F
#define GRED        0XFFE0
#define GBLUE       0X07FF
#define BROWN       0XBC40
#define BRRED       0XFC07
#define DARKBLUE    0X01CF
#define LIGHTBLUE   0X7D7C
#define GRAYBLUE    0X5458
#define LIGHTGREEN  0X841F
#define LGRAY       0XC618
#define LGRAYBLUE   0XA651
#define LBBLUE      0X2B12

/*******************************************************************************
 * ST7789 레지스터 정의
 ******************************************************************************/
#define ST7789_NOP     0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID   0x04
#define ST7789_RDDST   0x09

#define ST7789_SLPIN   0x10
#define ST7789_SLPOUT  0x11
#define ST7789_PTLON   0x12
#define ST7789_NORON   0x13

#define ST7789_INVOFF  0x20
#define ST7789_INVON   0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON  0x29
#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_RAMRD   0x2E

#define ST7789_PTLAR   0x30
#define ST7789_COLMOD  0x3A
#define ST7789_MADCTL  0x36

/* MADCTL 레지스터 비트 */
#define ST7789_MADCTL_MY  0x80  /* Page Address Order */
#define ST7789_MADCTL_MX  0x40  /* Column Address Order */
#define ST7789_MADCTL_MV  0x20  /* Page/Column Order */
#define ST7789_MADCTL_ML  0x10  /* Line Address Order */
#define ST7789_MADCTL_RGB 0x00  /* RGB/BGR Order */

#define ST7789_RDID1   0xDA
#define ST7789_RDID2   0xDB
#define ST7789_RDID3   0xDC
#define ST7789_RDID4   0xDD

/* 색상 모드 */
#define ST7789_COLOR_MODE_16bit 0x55    /* RGB565 (16bit) */
#define ST7789_COLOR_MODE_18bit 0x66    /* RGB666 (18bit) */

/*******************************************************************************
 * 하드웨어 제어 매크로
 ******************************************************************************/
#define ST7789_RST_Clr() HAL_GPIO_WritePin(ST7789_RST_PORT, ST7789_RST_PIN, GPIO_PIN_RESET)
#define ST7789_RST_Set() HAL_GPIO_WritePin(ST7789_RST_PORT, ST7789_RST_PIN, GPIO_PIN_SET)

#define ST7789_DC_Clr() HAL_GPIO_WritePin(ST7789_DC_PORT, ST7789_DC_PIN, GPIO_PIN_RESET)
#define ST7789_DC_Set() HAL_GPIO_WritePin(ST7789_DC_PORT, ST7789_DC_PIN, GPIO_PIN_SET)

#ifndef CFG_NO_CS
#define ST7789_Select() HAL_GPIO_WritePin(ST7789_CS_PORT, ST7789_CS_PIN, GPIO_PIN_RESET)
#define ST7789_UnSelect() HAL_GPIO_WritePin(ST7789_CS_PORT, ST7789_CS_PIN, GPIO_PIN_SET)
#else
#define ST7789_Select() asm("nop")
#define ST7789_UnSelect() asm("nop")
#endif

#define ABS(x) ((x) > 0 ? (x) : -(x))

/*******************************************************************************
 * 함수 프로토타입
 ******************************************************************************/

/* 기본 함수 */
void ST7789_Init(void);
void ST7789_SetRotation(uint8_t m);
void ST7789_Fill_Color(uint16_t color);
void ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7789_Fill(uint16_t xSta, uint16_t ySta, uint16_t xEnd, uint16_t yEnd, uint16_t color);
void ST7789_DrawPixel_4px(uint16_t x, uint16_t y, uint16_t color);

/* 그래픽 함수 */
void ST7789_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void ST7789_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void ST7789_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);
void ST7789_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data);
void ST7789_InvertColors(uint8_t invert);

/* 텍스트 함수 */
void ST7789_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor);
void ST7789_WriteString(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, uint16_t bgcolor);

/* 확장 그래픽 함수 */
void ST7789_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7789_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color);
void ST7789_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color);
void ST7789_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);

/* 명령 함수 */
void ST7789_TearEffect(uint8_t tear);

/* 테스트 함수 */
void ST7789_Test(void);

#ifndef ST7789_ROTATION
    #error "ST7789_ROTATION must be defined! (0-3)"
#endif

#endif /* __ST7789_H */
