/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : IVAS 기능별 단독 테스트 (LCD / LRF / GPS / IMU)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7789.h"
#include "fonts.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "magcalib.h"
#include "gps_neo_m8n.h"
#include "lrf.h"
#include "lora.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ===========================================================================
 *   ▼▼▼  테스트 모드 선택 : 이 숫자 하나만 바꿔서 빌드하세요  ▼▼▼
 *
 *     0 = LCD / 백라이트 테스트  (컬러바 + 밝기 페이드)
 *     1 = 레이저 거리측정        (화면 중앙 빨간 조준점 + 하단 거리 m)
 *     2 = GPS                    (위도 / 경도 / 위성수 / UTC)
 *     3 = IMU                    (YAW 나침반 그래픽)
 *     4 = 레이저 진단           (수신 바이트/프레임/RAW hex + 보레이트 자동탐색)
 *     5 = LoRa                  (RFM9x 송수신 / RSSI / SNR)
 *     6 = LoRa GPS              (A4 바이너리 : POLL 수신시 GPS 응답 + 웨이포인트)
 *     7 = 랜덤 좌표             (2초마다 랜덤 위/경도 표시, 표시 전용)
 * =========================================================================== */
#define TEST_MODE        1

/* --- 백라이트 밝기 0~100 [%] (PB1 / TIM3_CH4) --- */
#define BL_BRIGHTNESS    100

/* --- 레이저 모듈 (USART2 : PA2=TX, PA3=RX, PA1=EN) --- */
#define LRF_EN_GPIO_Port GPIOA
#define LRF_EN_Pin       GPIO_PIN_1
#define LRF_BAUDRATE     9600     /* 이 모듈은 9600 으로 확정. 바꾸지 말 것 */
#define LRF_CONTINUOUS   0        /* 1 = 연속측정, 0 = 1초마다 단발측정 (전류 부담 낮음) */
#define LRF_ENABLE_LASER 1        /* 0 = EN 을 Low 로 고정 (레이저 끔) - 전원 문제 판별용 */
#define LRF_SHOW_DEBUG   1        /* 1 = 화면 하단에 수신 카운터 표시 */
#define LRF_DIAG_AUTOBAUD 1       /* MODE 4 에서 9600/115200 자동 번갈아 시도 */

/* --- LoRa RFM9x (SPI1 공유 : SCK=PA5, MISO=PB4, MOSI=PA7, CS=PB12) --- */
#define LORA_TX_PERIOD_MS 3000    /* 이 주기로 테스트 패킷 송신. 0 이면 수신 전용 */
#define LORA_TX_TIMEOUT   2000

/* --- MODE 1 조준점 색 ---
   RGB565 에서 RED(0xF800) 은 이미 R 성분이 최대치라 그 자체로는 더 밝게
   못 만든다. G/B 를 조금 섞어야 실제로 밝아진다.
     0xF800  순수 적색 (기존)      상대휘도 54
     0xF9C6  밝은 적색  R31 G14 B6      98   <-- 현재 설정
     0xFACB  더 밝은 적색 R31 G22 B11   124  (연분홍 쪽)
     0xFC07  주황 쪽 (BRRED)            151  */
#define RET_COLOR         0xF9C6

/* --- 화면 --- */
#define SCR_W            ST7789_WIDTH
#define SCR_H            ST7789_HEIGHT
#define DEG2RAD          0.01745329252f

#if   (TEST_MODE == 0)
  #define MODE_NAME      "LCD TEST"
#elif (TEST_MODE == 1)
  #define MODE_NAME      "LRF"
#elif (TEST_MODE == 2)
  #define MODE_NAME      "GPS"
#elif (TEST_MODE == 3)
  #define MODE_NAME      "IMU YAW"
#elif (TEST_MODE == 4)
  #define MODE_NAME      "LRF DIAG"
#elif (TEST_MODE == 5)
  #define MODE_NAME      "LoRa"
#elif (TEST_MODE == 6)
  #define MODE_NAME      "LoRa GPS"
#elif (TEST_MODE == 7)
  #define MODE_NAME      "POS"
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
MPU6050_Data_t        mpu_data;
MPU6050_Calibration_t mpu_cal;
HMC5883L_Data_t       mag_data;
GPS_Handle_t          gps;
LRF_Handle_t          lrf;

/* UART 수신 바이트 (GPS용) */
uint8_t gps_rx_byte;

/* 센서 검출 상태 */
uint8_t imu_ok = 0, mag_ok = 0;

/* 부팅 시 리셋 원인 (RCC->CSR 을 지우기 전에 래치해 둔다) */
uint32_t boot_rccsr = 0;
char     boot_cause[28] = "?";

/* 자기장 캘리브레이션 데이터 (캘리브레이션 후 이 값을 하드코딩) */
MagCalib_t mag_cal = {
    .b = {-0.428939, 0.197259, 0.113888},
    .A = {
    		{5.457004, 0.058187, -0.229328},
    		{0.058187, 3.929403, 0.029632},
    		{-0.229328, 0.029632, 3.305897}
    }
};


/* LCD 표시용 버퍼 */
char lcd_buf[64];
/* LCD 변수 표시용 버퍼 */
char buf[32];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */


int _write(int file, char *ptr, int len)
{
    (void)file;
    for (int i = 0; i < len; i++)
    {
        ITM_SendChar(*ptr++);
    }
    return len;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ==========================================================================
 *  원형 표시영역 (지름 240px, 화면 정중앙) + 공통 유틸
 *
 *  물리 패널은 240x240 정사각형이지만 UI 는 그 안에 내접하는 지름 240 원
 *  안쪽에만 그린다. 네 모서리는 항상 검게 비워 둔다.
 *  가로 한 줄의 사용 가능 폭은 y 에 따라 달라진다 :  2 * sqrt(R^2 - dy^2)
 * ========================================================================== */
#define UI_CX   (SCR_W / 2)     /* 120 */
#define UI_CY   (SCR_H / 2)     /* 120 */
#define UI_R    118             /* 표시 가능 반지름 (테두리 여유 2px) */

/* 백라이트 밝기 설정 (0~100%) */
static void BL_Set(uint8_t pct)
{
    if (pct > 100) pct = 100;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)pct * 655u);
}

/* 중심에서 dy 만큼 떨어진 가로줄의 반폭 */
__attribute__((unused))
static uint16_t UI_HalfWidth(int dy)
{
    int v = UI_R * UI_R - dy * dy;
    if (v <= 0) return 0;
    return (uint16_t)sqrtf((float)v);
}

/* 원 안쪽만 색으로 채움 (행 단위라 DrawFilledCircle 보다 훨씬 빠름) */
__attribute__((unused))
static void UI_FillBand(int y0, int y1, uint16_t color)
{
    for (int y = y0; y <= y1; y++) {
        if (y < 0 || y >= SCR_H) continue;
        uint16_t hw = UI_HalfWidth(y - UI_CY);
        if (hw == 0) continue;
        ST7789_Fill((uint16_t)(UI_CX - hw), (uint16_t)y,
                    (uint16_t)(UI_CX + hw), (uint16_t)y, color);
    }
}

/* 원형 베젤 (표시영역 경계) */
static void UI_Bezel(void)
{
    ST7789_DrawCircle(UI_CX, UI_CY, UI_R + 1, GRAY);
    ST7789_DrawCircle(UI_CX, UI_CY, UI_R,     LGRAY);
}

/* 화면 지우고 베젤 다시 그리기 */
__attribute__((unused))
static void UI_Clear(void)
{
    ST7789_Fill_Color(BLACK);
    UI_Bezel();
}

/* 문자열을 가로 중앙 정렬해서 출력 */
static void Str_Center(uint16_t y, const char *s, FontDef f, uint16_t fg, uint16_t bg)
{
    uint16_t w = (uint16_t)(strlen(s) * f.width);
    uint16_t x = (w >= SCR_W) ? 0 : (uint16_t)((SCR_W - w) / 2);
    ST7789_WriteString(x, y, s, f, fg, bg);
}

/* 폭을 w 문자로 고정해서 중앙 정렬 출력.
   문자열 길이가 바뀌어도 이전 글자가 남지 않는다. */
__attribute__((unused))
static void Str_CenterW(uint16_t y, const char *s, FontDef f,
                        uint16_t fg, uint16_t bg, uint8_t w)
{
    char pad[40];
    uint8_t n = (uint8_t)strlen(s);
    if (w > (uint8_t)(sizeof(pad) - 1)) w = (uint8_t)(sizeof(pad) - 1);
    if (n > w) n = w;
    memcpy(pad, s, n);
    memset(&pad[n], ' ', (size_t)(w - n));
    pad[w] = '\0';
    Str_Center(y, pad, f, fg, bg);
}

/* double 좌표를 소수점 6자리 문자열로 (nano.specs 에서 %f 는 동작하지 않음).
   항상 11글자 고정 폭이라 중앙정렬이 흔들리지 않는다. */
__attribute__((unused))
static void Fmt_Coord(char *out, size_t n, double v)
{
    char sign = ' ';
    if (v < 0) { sign = '-'; v = -v; }
    uint32_t ip = (uint32_t)v;
    uint32_t fp = (uint32_t)((v - (double)ip) * 1000000.0 + 0.5);
    if (fp >= 1000000u) { fp -= 1000000u; ip++; }
    snprintf(out, n, "%c%3lu.%06lu", sign, (unsigned long)ip, (unsigned long)fp);
}

/* RCC->CSR 리셋 플래그를 사람이 읽을 수 있는 문자열로 */
static void Fmt_ResetCause(char *out, size_t n, uint32_t csr)
{
    size_t k = 0;
    out[0] = '\0';
    #define ADD(flag, txt)  do { if ((csr) & (flag)) { \
        int w = snprintf(&out[k], n - k, "%s ", (txt)); \
        if (w > 0 && (size_t)w < n - k) k += (size_t)w; } } while (0)
    ADD(RCC_CSR_LPWRRSTF, "LPWR");
    ADD(RCC_CSR_WWDGRSTF, "WWDG");
    ADD(RCC_CSR_IWDGRSTF, "IWDG");
    ADD(RCC_CSR_SFTRSTF,  "SFT");
    ADD(RCC_CSR_PORRSTF,  "POR");
    ADD(RCC_CSR_PINRSTF,  "PIN");
    ADD(RCC_CSR_BORRSTF,  "BOR");
    #undef ADD
    if (out[0] == '\0') snprintf(out, n, "NONE");
}

/* 부팅 스플래시 - 리셋 원인을 함께 표시한다.
 * 이 화면이 반복해서 나타나면 MCU 가 리셋되고 있다는 뜻이고,
 * RESET 줄이 그 원인을 알려준다.
 *   POR BOR  : 전원 강하(브라운아웃). 레이저 모듈 전류/배선을 의심할 것
 *   PIN      : NRST 핀이 눌림 (디버거 / 배선 / 노이즈)
 *   IWDG WWDG: 워치독 (이 프로젝트는 사용 안 함)
 *   SFT      : 소프트웨어 리셋
 */
static void Screen_Splash(const char *title, const char *sub)
{
    char s[40];
    UI_Clear();
    Str_Center(76,  title, Font_16x26, WHITE, BLACK);
    Str_Center(110, sub,   Font_11x18, GRAY,  BLACK);
    snprintf(s, sizeof(s), "RST %s", boot_cause);
    Str_CenterW(150, s, Font_7x10, YELLOW, BLACK, 26);
    snprintf(s, sizeof(s), "RCC_CSR = %08lX", (unsigned long)boot_rccsr);
    Str_Center(164, s, Font_7x10, GRAY, BLACK);
}

/* ==========================================================================
 *  랜덤 좌표 생성기   (MODE 2 화면 하단 / MODE 7 에서 공용으로 쓴다)
 *
 *  아래 사각 범위 안에서 RND_PERIOD_MS 마다 위/경도를 무작위로 뽑는다.
 *      위도  37.3187701 ~ 37.3199373   (약 130 m)
 *      경도 127.1260417 ~ 127.1264007  (약  32 m)
 * ========================================================================== */
#define RND_PERIOD_MS   2000
#define RND_LAT_MIN      373187701      /*  37.3187701 x 1e7 */
#define RND_LAT_MAX      373199373      /*  37.3199373 x 1e7 */
#define RND_LON_MIN     1271260417      /* 127.1260417 x 1e7 */
#define RND_LON_MAX     1271264007      /* 127.1264007 x 1e7 */

static uint32_t rnd_state = 0x2545F491u;
static int32_t  rnd_lat   = RND_LAT_MIN;
static int32_t  rnd_lon   = RND_LON_MIN;

/* xorshift32 - rand() 를 링크하지 않으려고 직접 쓴다 */
__attribute__((unused))
static uint32_t Rnd_Next(void)
{
    uint32_t x = rnd_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rnd_state = x;
    return x;
}

/* lo ~ hi 사이의 값 하나 (양 끝 포함) */
__attribute__((unused))
static int32_t Rnd_Range(int32_t lo, int32_t hi)
{
    uint32_t span = (uint32_t)(hi - lo) + 1u;
    return lo + (int32_t)(Rnd_Next() % span);
}

__attribute__((unused))
static void Rnd_Seed(void)
{
    rnd_state ^= HAL_GetTick() * 2654435761u;       /* 매번 다른 수열로 */
    if (rnd_state == 0u) rnd_state = 0x2545F491u;   /* xorshift 는 0 이면 멈춘다 */
}

/* 주기가 되면 새 좌표를 만든다. 값이 갱신됐으면 true */
__attribute__((unused))
static bool Rnd_Tick(uint32_t now)
{
    static uint32_t t = 0;
    static bool     first = true;

    if (!first && ((now - t) < RND_PERIOD_MS)) return false;
    t = now;
    first = false;
    rnd_lat = Rnd_Range(RND_LAT_MIN, RND_LAT_MAX);
    rnd_lon = Rnd_Range(RND_LON_MIN, RND_LON_MAX);
    return true;
}

/* 도 x 1e7 정수를 "37.3188315" 형태로 (nano.specs 에 %f 가 없어 정수로 조립) */
__attribute__((unused))
static void Rnd_Fmt(char *out, size_t n, int32_t v)
{
    snprintf(out, n, "%ld.%07ld", (long)(v / 10000000L), (long)(v % 10000000L));
}

/* ==========================================================================
 *  MODE 0 : LCD / 백라이트 테스트  (원형 컬러 밴드 + 밝기 페이드)
 * ========================================================================== */
#if (TEST_MODE == 0)
static void Mode0_Init(void)
{
    static const uint16_t bars[6] = { RED, YELLOW, GREEN, CYAN, BLUE, WHITE };

    ST7789_Fill_Color(BLACK);
    for (uint8_t i = 0; i < 6; i++) {
        UI_FillBand(2 + i * 39, 2 + (i + 1) * 39 - 1, bars[i]);
    }
    UI_FillBand(102, 140, BLACK);      /* 글자 자리 비우기 */
    Str_Center(106, "LCD TEST", Font_16x26, WHITE, BLACK);
    UI_Bezel();
}

static void Mode0_Loop(void)
{
    /* 백라이트 페이드 인/아웃 */
    for (int p = 10; p <= 100; p += 5) { BL_Set((uint8_t)p); HAL_Delay(30); }
    for (int p = 100; p >= 10; p -= 5) { BL_Set((uint8_t)p); HAL_Delay(30); }
}
#endif /* TEST_MODE == 0 */

/* ==========================================================================
 *  MODE 1 : 레이저 거리측정  (원형 조준경 화면)
 *
 *      ....................
 *    ..        LRF       ..      <- 상단 라벨 + 수신표시등
 *   .          |          .
 *   .      ---- ----      .      <- 중앙 빨간 조준점
 *   .          |          .
 *    ..     123.4 m     ..       <- 측정 거리
 *      ....RX0 F0 E0.....        <- 진단 카운터
 * ========================================================================== */
#if (TEST_MODE == 1)
#define RET_CX   UI_CX
#define RET_CY   UI_CY
#define RET_R    34

static void Reticle_Draw(void)
{
    /* 바깥 원 */
    ST7789_DrawCircle(RET_CX, RET_CY, RET_R,     RET_COLOR);
    ST7789_DrawCircle(RET_CX, RET_CY, RET_R - 1, RET_COLOR);

    /* 십자선 (중앙은 비워둠) */
    ST7789_DrawLine(RET_CX, RET_CY - RET_R - 10, RET_CX, RET_CY - 10, RET_COLOR);
    ST7789_DrawLine(RET_CX, RET_CY + 10, RET_CX, RET_CY + RET_R + 10, RET_COLOR);
    ST7789_DrawLine(RET_CX - RET_R - 10, RET_CY, RET_CX - 10, RET_CY, RET_COLOR);
    ST7789_DrawLine(RET_CX + 10, RET_CY, RET_CX + RET_R + 10, RET_CY, RET_COLOR);

    /* 중심 점 */
    ST7789_DrawFilledCircle(RET_CX, RET_CY, 2, RET_COLOR);
}

/* 하단 진단 밴드 : 수신 바이트 / 프레임 / 깨진 프레임 */
static void Mode1_Debug(void)
{
#if LRF_SHOW_DEBUG
    static uint32_t t = 0, pb = 0xFFFFFFFF, pf = 0xFFFFFFFF, pe = 0xFFFFFFFF;
    char s[32];
    if (HAL_GetTick() - t < 400) return;
    t = HAL_GetTick();
    if (lrf.rx_bytes == pb && lrf.rx_frames == pf && lrf.bad_frames == pe) return;
    pb = lrf.rx_bytes; pf = lrf.rx_frames; pe = lrf.bad_frames;

    snprintf(s, sizeof(s), "RX%lu F%lu E%lu",
             (unsigned long)(pb > 99999u ? 99999u : pb),
             (unsigned long)(pf > 999u   ? 999u   : pf),
             (unsigned long)(pe > 999u   ? 999u   : pe));
    Str_CenterW(46, s, Font_7x10, (pb == 0) ? RED : GREEN, BLACK, 20);
#endif
}

static void Mode1_Init(void)
{
    /* 화면을 먼저 완성해 두고 그 다음에 모듈을 켠다.
       모듈 기동 중에 문제가 생겨도 화면이 남아 있어야 원인이 보인다. */
    UI_Clear();
    Str_Center(22, "LRF", Font_11x18, GREEN, BLACK);
    Reticle_Draw();
    Str_CenterW(168, "  --.- m", Font_16x26, WHITE, BLACK, 8);
#if !LRF_ENABLE_LASER
    Str_Center(60, "EN = LOW", Font_7x10, RED, BLACK);
#endif

    /* 수신부터 열고 -> EN High -> 부팅 대기 -> 명령 */
    /* 하단 랜덤 좌표 구획 */
    ST7789_DrawLine(56, 195, SCR_W - 56, 195, GRAY);
    Rnd_Seed();

    LRF_Init(&lrf, &huart2, LRF_EN_GPIO_Port, LRF_EN_Pin,
             LRF_ENABLE_LASER ? true : false);
    HAL_Delay(1000);                 /* 모듈 부팅("init ok") 대기 */
    LRF_Reset(&lrf);
    HAL_Delay(300);
#if LRF_CONTINUOUS
    LRF_ContinuousStart(&lrf);
#endif
}

static void Mode1_Loop(void)
{
    static int32_t  shown   = -2;          /* 마지막으로 그린 값 (0.1m) */
    static uint32_t t_meas  = 0;
    static uint32_t t_rxled = 0;
    uint32_t now = HAL_GetTick();
    char s[24], c[16];

#if LRF_CONTINUOUS
    /* 3초 이상 무응답이면 연속측정 명령 재전송 */
    if (now - lrf.last_rx_tick > 3000) {
        LRF_ContinuousStart(&lrf);
        lrf.last_rx_tick = now;
    }
#else
    /* 1초마다 단발 측정 요청 */
    if (now - t_meas >= 1000) {
        t_meas = now;
        LRF_MeasureSingle(&lrf);
    }
#endif
    (void)t_meas;

    Mode1_Debug();

    if (lrf.updated) {
        lrf.updated = false;

        int32_t v = lrf.valid ? (int32_t)LRF_GetDistance_dm(&lrf) : -1;
        if (v != shown) {
            shown = v;
            if (v >= 0) snprintf(s, sizeof(s), "%4ld.%ld m", (long)(v / 10), (long)(v % 10));
            else        snprintf(s, sizeof(s), "  --.- m");
            Str_CenterW(168, s, Font_16x26, lrf.valid ? YELLOW : GRAY, BLACK, 8);
        }
        t_rxled = now;
    }

    /* 수신 표시등 (원 안쪽 상단 우측) */
    static uint8_t led = 0xFF;
    uint8_t on = (now - t_rxled < 300) ? 1 : 0;
    if (on != led) {
        led = on;
        ST7789_DrawFilledCircle(UI_CX + 42, 35, 5, on ? GREEN : 0x2104);
    }

    /* --- 하단 : 랜덤 좌표 (2초마다 갱신, MODE 2/7 과 같은 생성기) --- */
    if (Rnd_Tick(now)) {
        Rnd_Fmt(c, sizeof(c), rnd_lat);
        snprintf(s, sizeof(s), "LAT %s", c);
        Str_CenterW(198, s, Font_7x10, CYAN, BLACK, 15);

        Rnd_Fmt(c, sizeof(c), rnd_lon);
        snprintf(s, sizeof(s), "LON %s", c);
        Str_CenterW(210, s, Font_7x10, CYAN, BLACK, 15);
    }

    HAL_Delay(30);
}
#endif /* TEST_MODE == 1 */

/* ==========================================================================
 *  MODE 2 : GPS  (위도 / 경도)  - 원형 배치, 각 줄 중앙정렬
 * ========================================================================== */
#if (TEST_MODE == 2)
#define GPS_ROW0   68
#define GPS_PITCH  24
#define GPS_COLS   15          /* 모든 줄을 15글자 고정폭으로 */

static void Mode2_Init(void)
{
    GPS_Init(&gps, &huart1);
    HAL_UART_Receive_IT(&huart1, &gps_rx_byte, 1);
    HAL_Delay(600);

    UI_Clear();
    Str_Center(30, "GPS", Font_16x26, GREEN, BLACK);
    ST7789_DrawLine(56, 60, SCR_W - 56, 60, GRAY);

    /* 하단 랜덤 좌표 구획 */
    ST7789_DrawLine(56, 182, SCR_W - 56, 182, GRAY);
    Rnd_Seed();
}

static void Mode2_Loop(void)
{
    static uint32_t t_ui = 0;
    char s[32], c[16];

    /* NMEA 문장 파싱 (수신은 인터럽트로 계속 들어옴) */
    GPS_Process(&gps);

    uint32_t now = HAL_GetTick();
    if (now - t_ui < 250) return;
    t_ui = now;

    if (gps.data.valid) {
        Fmt_Coord(c, sizeof(c), gps.data.latitude);
        snprintf(s, sizeof(s), "LAT%s", c);
        Str_CenterW(GPS_ROW0 + 0 * GPS_PITCH, s, Font_11x18, WHITE, BLACK, GPS_COLS);

        Fmt_Coord(c, sizeof(c), gps.data.longitude);
        snprintf(s, sizeof(s), "LON%s", c);
        Str_CenterW(GPS_ROW0 + 1 * GPS_PITCH, s, Font_11x18, WHITE, BLACK, GPS_COLS);
    } else {
        Str_CenterW(GPS_ROW0 + 0 * GPS_PITCH, "LAT   NO FIX", Font_11x18, RED, BLACK, GPS_COLS);
        Str_CenterW(GPS_ROW0 + 1 * GPS_PITCH, "LON   NO FIX", Font_11x18, RED, BLACK, GPS_COLS);
    }

    snprintf(s, sizeof(s), "ALT %6d m", (int)gps.data.altitude);
    Str_CenterW(GPS_ROW0 + 2 * GPS_PITCH, s, Font_11x18, WHITE, BLACK, GPS_COLS);

    snprintf(s, sizeof(s), "SAT %2u  FIX %u", gps.data.satellites, gps.data.fix_quality);
    Str_CenterW(GPS_ROW0 + 3 * GPS_PITCH, s, Font_11x18, WHITE, BLACK, GPS_COLS);

    snprintf(s, sizeof(s), "UTC %02u:%02u:%02u",
             gps.data.hour, gps.data.minute, gps.data.second);
    Str_CenterW(GPS_ROW0 + 4 * GPS_PITCH, s, Font_11x18, WHITE, BLACK, GPS_COLS);

    /* --- 하단 : 랜덤 좌표 (2초마다 갱신, 실제 GPS 와 구분되게 노란색) --- */
    if (Rnd_Tick(now)) {
        Rnd_Fmt(c, sizeof(c), rnd_lat);
        snprintf(s, sizeof(s), "LAT %s", c);
        Str_CenterW(188, s, Font_7x10, CYAN, BLACK, 15);

        Rnd_Fmt(c, sizeof(c), rnd_lon);
        snprintf(s, sizeof(s), "LON %s", c);
        Str_CenterW(202, s, Font_7x10, CYAN, BLACK, 15);
    }
}
#endif /* TEST_MODE == 2 */

/* ==========================================================================
 *  MODE 3 : IMU  (YAW 나침반 그래픽) - 원형 표시영역에 맞춰 축소 배치
 * ========================================================================== */
#if (TEST_MODE == 3)
#define CMP_CX   UI_CX
#define CMP_CY   106
#define CMP_R    70
#define CMP_RL   50      /* N/E/S/W 글자 반지름 */
#define CMP_RN   40      /* 북쪽 지시선 길이 */

static const char *CARD_CH[4] = { "N", "E", "S", "W" };

/* 다이얼(글자 + 북쪽 지시선)을 heading 에 맞춰 그림. color=BLACK 이면 지우기 */
static void Compass_Dial(float heading, uint16_t col_n, uint16_t col_o)
{
    for (uint8_t i = 0; i < 4; i++) {
        float a  = ((float)i * 90.0f - heading) * DEG2RAD;
        int   lx = CMP_CX + (int)(CMP_RL * sinf(a)) - 5;
        int   ly = CMP_CY - (int)(CMP_RL * cosf(a)) - 9;
        ST7789_WriteString((uint16_t)lx, (uint16_t)ly, CARD_CH[i],
                           Font_11x18, (i == 0) ? col_n : col_o, BLACK);
    }
    /* 북쪽 지시선 */
    float an = (-heading) * DEG2RAD;
    ST7789_DrawLine(CMP_CX, CMP_CY,
                    (uint16_t)(CMP_CX + (int)(CMP_RN * sinf(an))),
                    (uint16_t)(CMP_CY - (int)(CMP_RN * cosf(an))), col_n);
}

static void Mode3_Init(void)
{
    UI_Clear();
    Str_Center(100, "calibrating...", Font_11x18, GRAY, BLACK);

    imu_ok = (MPU6050_Init(&hi2c1) == HAL_OK);
    mag_ok = (HMC5883L_Init(&hi2c1) == HAL_OK);
    if (imu_ok) MPU6050_Calibrate_Gyro(&hi2c1, &mpu_cal, 500);

    UI_Clear();

    /* 고정 요소 : 바깥 링 */
    ST7789_DrawCircle(CMP_CX, CMP_CY, CMP_R,     WHITE);
    ST7789_DrawCircle(CMP_CX, CMP_CY, CMP_R - 1, GRAY);
    /* 고정 요소 : 정면(기수) 지시 삼각형 */
    ST7789_DrawFilledTriangle(CMP_CX, CMP_CY - CMP_R + 2,
                              CMP_CX - 7, CMP_CY - CMP_R - 10,
                              CMP_CX + 7, CMP_CY - CMP_R - 10, RED);
    ST7789_DrawFilledCircle(CMP_CX, CMP_CY, 3, WHITE);

    if (!mag_ok) Str_Center(182, "NO MAG", Font_16x26, RED, BLACK);
}

static void Mode3_Loop(void)
{
    static float    prev_h = 9999.0f;
    static float    filt_h = 0.0f;
    static uint8_t  first  = 1;
    static uint32_t t_ui   = 0;
    char s[16];

    if (!mag_ok) { HAL_Delay(200); return; }

    if (imu_ok) MPU6050_Read_All_Calibrated(&hi2c1, &mpu_data, &mpu_cal);
    HMC5883L_Read_All(&hi2c1, &mag_data, mpu_data.roll, mpu_data.pitch, &mag_cal);

    /* 각도 wrap 을 고려한 저역통과 필터 */
    float h = mag_data.heading;
    if (first) { filt_h = h; first = 0; }
    else {
        float d = h - filt_h;
        while (d >  180.0f) d -= 360.0f;
        while (d < -180.0f) d += 360.0f;
        filt_h += 0.25f * d;
        if (filt_h < 0.0f)     filt_h += 360.0f;
        if (filt_h >= 360.0f)  filt_h -= 360.0f;
    }

    uint32_t now = HAL_GetTick();
    if (now - t_ui < 80) return;
    t_ui = now;

    /* 2도 이상 변했을 때만 다시 그림 */
    float dd = filt_h - prev_h;
    while (dd >  180.0f) dd -= 360.0f;
    while (dd < -180.0f) dd += 360.0f;
    if (prev_h > 999.0f || fabsf(dd) >= 2.0f) {
        if (prev_h <= 999.0f) Compass_Dial(prev_h, BLACK, BLACK);   /* 이전 것 지우기 */
        Compass_Dial(filt_h, RED, WHITE);
        ST7789_DrawFilledCircle(CMP_CX, CMP_CY, 3, WHITE);   /* 중심 허브 복구 */
        prev_h = filt_h;

        static const char *NAME[8] = {"N ","NE","E ","SE","S ","SW","W ","NW"};
        uint8_t k = (uint8_t)(((int)(filt_h + 22.5f) / 45) & 7);
        snprintf(s, sizeof(s), "%3d %s", (int)filt_h, NAME[k]);
        Str_CenterW(182, s, Font_16x26, YELLOW, BLACK, 6);
    }
}
#endif /* TEST_MODE == 3 */

/* ==========================================================================
 *  MODE 4 : 레이저 모듈 진단 (원형 배치)
 *    - 리셋 원인, 보레이트, EN 핀 상태
 *    - 수신 바이트 / 정상 프레임 / 깨진 프레임 카운터
 *    - 최근 수신 16바이트 RAW hex
 *    - LRF_DIAG_AUTOBAUD=1 이면 5초마다 115200 <-> 9600 을 번갈아 시도
 * ========================================================================== */
#if (TEST_MODE == 4)
static const uint32_t DIAG_BAUD[2] = { 115200, 9600 };

static void Mode4_Init(void)
{
    UI_Clear();
    Str_Center(38, "LRF DIAG", Font_11x18, GREEN, BLACK);

    LRF_Init(&lrf, &huart2, LRF_EN_GPIO_Port, LRF_EN_Pin,
             LRF_ENABLE_LASER ? true : false);

    Str_Center(194, "RX 0  = wiring/power", Font_7x10, GRAY, BLACK);
    Str_Center(206, "FRM 0 = baud",         Font_7x10, GRAY, BLACK);
}

static void Mode4_Loop(void)
{
    static uint8_t  bi = 0;
    static uint32_t t_baud = 0, t_cmd = 0, t_ui = 0, base_bytes = 0;
    char s[40], hex[LRF_RAW_MAX * 3 + 8];
    uint32_t now = HAL_GetTick();

#if LRF_DIAG_AUTOBAUD
    if (now - t_baud >= 5000) {
        t_baud     = now;
        bi        ^= 1u;
        base_bytes = lrf.rx_bytes;
        LRF_SetBaud(&lrf, DIAG_BAUD[bi]);
    }
#endif

    /* 1초마다 단발 측정 요청 */
    if (now - t_cmd >= 1000) { t_cmd = now; LRF_MeasureSingle(&lrf); }

    if (now - t_ui < 200) return;
    t_ui = now;

    snprintf(s, sizeof(s), "RST %s", boot_cause);
    Str_CenterW(66, s, Font_7x10, YELLOW, BLACK, 26);

    snprintf(s, sizeof(s), "BAUD %lu  EN %s",
             (unsigned long)lrf.huart->Init.BaudRate,
             (HAL_GPIO_ReadPin(LRF_EN_GPIO_Port, LRF_EN_Pin) == GPIO_PIN_SET) ? "HI" : "LO");
    Str_CenterW(82, s, Font_7x10, WHITE, BLACK, 26);

    snprintf(s, sizeof(s), "RX %lu byte", (unsigned long)lrf.rx_bytes);
    Str_CenterW(98, s, Font_7x10, (lrf.rx_bytes > base_bytes) ? GREEN : RED, BLACK, 26);

    snprintf(s, sizeof(s), "FRM %lu   BAD %lu",
             (unsigned long)lrf.rx_frames, (unsigned long)lrf.bad_frames);
    Str_CenterW(114, s, Font_7x10, lrf.rx_frames ? GREEN : GRAY, BLACK, 26);

    if (lrf.valid) {
        uint16_t d = LRF_GetDistance_dm(&lrf);
        snprintf(s, sizeof(s), "DIST %lu.%lu m",
                 (unsigned long)(d / 10), (unsigned long)(d % 10));
    } else {
        snprintf(s, sizeof(s), "DIST --.- m");
    }
    Str_CenterW(130, s, Font_7x10, CYAN, BLACK, 26);

    Str_Center(150, "RAW (last 16 bytes)", Font_7x10, GRAY, BLACK);
    LRF_RawToHex(&lrf, hex, sizeof(hex), LRF_RAW_MAX);
    Str_CenterW(164, hex, Font_7x10, WHITE, BLACK, 24);
    Str_CenterW(176, (strlen(hex) > 24) ? hex + 24 : "", Font_7x10, WHITE, BLACK, 24);
}
#endif /* TEST_MODE == 4 */

/* ==========================================================================
 *  MODE 5 : LoRa (RFM9x / SX1276)
 *
 *  ESP32 레퍼런스 구현(lora.c)과 무선 파라미터가 동일하므로 그대로 통신된다.
 *    922.1 MHz / BW 125 kHz / SF7 / CR 4/5 / explicit header / CRC on
 *    프리앰블 8, 싱크워드 0x12 (양쪽 다 기본값)
 *
 *  기본 동작 : 계속 수신 대기하다가 LORA_TX_PERIOD_MS 마다 테스트 패킷 송신
 * ========================================================================== */
#if (TEST_MODE == 5)
static uint8_t  lora_rx[64];
static uint32_t lora_tx_cnt, lora_rx_cnt, lora_err_cnt;
static char     lora_last[24] = "-";

/* 수신 페이로드를 화면에 찍을 수 있는 ASCII 로 (비출력 문자는 '.') */
static void LoRa_PayloadToText(const uint8_t *p, uint8_t n, char *out, size_t cap)
{
    size_t i, m = (n < (cap - 1)) ? n : (cap - 1);
    for (i = 0; i < m; i++) {
        out[i] = (p[i] >= 0x20 && p[i] < 0x7F) ? (char)p[i] : '.';
    }
    out[m] = '\0';
}

static void Mode5_Init(void)
{
    UI_Clear();
    Str_Center(30, "LoRa", Font_11x18, GREEN, BLACK);
    Str_Center(54, "922.1MHz SF7 BW125", Font_7x10, GRAY, BLACK);

    if (LoRa_Init() != HAL_OK) {
        char s[32];
        snprintf(s, sizeof(s), "VER 0x%02X  FAIL", LoRa_ChipVersion());
        Str_Center(96,  s, Font_11x18, RED, BLACK);
        Str_Center(126, "check MISO(PB4)",  Font_7x10, GRAY, BLACK);
        Str_Center(138, "CS(PB12) RST EN",  Font_7x10, GRAY, BLACK);
        Str_Center(150, "3V3 power",        Font_7x10, GRAY, BLACK);
        return;
    }

    ST7789_DrawLine(56, 70, SCR_W - 56, 70, GRAY);
}

static void Mode5_Loop(void)
{
    static uint32_t t_tx = 0, t_ui = 0;
    uint32_t now = HAL_GetTick();
    char s[32];

    if (!LoRa_IsReady()) { HAL_Delay(200); return; }

    /* ---- 수신 ---- */
    uint8_t len = 0, irq = 0;
    int r = LoRa_ReceivePacket(lora_rx, &len, &irq);
    if (r == 1 && len > 0) {
        lora_rx_cnt++;
        LoRa_PayloadToText(lora_rx, len, lora_last, sizeof(lora_last));
        LoRa_RxSet();                       /* 다시 연속 수신으로 */
    } else if (r < 0) {
        lora_err_cnt++;
        LoRa_RxSet();
    }

    /* ---- 주기 송신 ---- */
#if (LORA_TX_PERIOD_MS > 0)
    if (now - t_tx >= LORA_TX_PERIOD_MS) {
        t_tx = now;
        int n = snprintf(s, sizeof(s), "IVAS %lu", (unsigned long)(lora_tx_cnt + 1));
        if (n > 0 && LoRa_SendPacket((const uint8_t *)s, (uint8_t)n, LORA_TX_TIMEOUT)) {
            lora_tx_cnt++;
        }
        LoRa_RxSet();                       /* 송신 후에는 STANDBY 이므로 복귀 필수 */
    }
#endif
    (void)t_tx;

    /* ---- 화면 ---- */
    if (now - t_ui < 250) return;
    t_ui = now;

    snprintf(s, sizeof(s), "VER 0x%02X", LoRa_ChipVersion());
    Str_CenterW(82, s, Font_7x10, GREEN, BLACK, 22);

    snprintf(s, sizeof(s), "TX %lu   RX %lu   E %lu",
             (unsigned long)lora_tx_cnt, (unsigned long)lora_rx_cnt,
             (unsigned long)lora_err_cnt);
    Str_CenterW(100, s, Font_7x10, WHITE, BLACK, 26);

    snprintf(s, sizeof(s), "RSSI %d dBm", (int)LoRa_PacketRssiDbm());
    Str_CenterW(118, s, Font_7x10, CYAN, BLACK, 22);

    int16_t snr = LoRa_PacketSnrX100();
    int sgn = (snr < 0) ? -1 : 1;
    int mag = (snr < 0) ? -snr : snr;
    snprintf(s, sizeof(s), "SNR %s%d.%02d dB", (sgn < 0) ? "-" : "", mag / 100, mag % 100);
    Str_CenterW(134, s, Font_7x10, CYAN, BLACK, 22);

    Str_Center(158, "last RX", Font_7x10, GRAY, BLACK);
    Str_CenterW(172, lora_last, Font_11x18, YELLOW, BLACK, 16);
}
#endif /* TEST_MODE == 5 */

/* ==========================================================================
 *  MODE 6 : LoRa GPS  -  A4 바이너리 프로토콜 (기지국 TTGO / 주작 GUI 호환)
 *
 *  [공통 패킷]  A4 SRC DST TYPE SEQ_L SEQ_H LEN PAYLOAD...
 *      [0] 0xA4 고정 헤더
 *      [1] SRC   송신 장치 ID
 *      [2] DST   수신 장치 ID
 *      [3] TYPE  메시지 종류
 *    [4:5] SEQ   uint16 little-endian
 *      [6] LEN   payload 길이
 *      [7~]      payload
 *    다중 바이트 정수는 전부 little-endian.
 *
 *  [수신] 기지국(0xFF) -> 헬멧(0xFD)
 *    POLL      A4 FF FD 01 SEQ_L SEQ_H 00                        (7 byte)
 *    WAYPOINT  A4 FF FD 02 SEQ_L SEQ_H LEN COUNT LAT[4] LON[4] ...
 *              LEN   = 1 + COUNT*8,  전체 = 7 + LEN = 8 + COUNT*8
 *              COUNT = 1~15
 *              i 번째 :  LAT = pkt[8 + i*8],  LON = pkt[12 + i*8]
 *
 *  [송신] 헬멧(0xFD) -> 기지국(0xFF)
 *    UAV_GPS   A4 FD FF 10 SEQ_L SEQ_H 09 LAT[4] LON[4] VALID    (16 byte)
 *    ACK       A4 FD FF 03 SEQ_L SEQ_H 02 02 00                  (9 byte)
 *              PAYLOAD[0]=0x02 대상 명령(WAYPOINT), PAYLOAD[1]=0x00 성공
 *
 *  LAT/LON 은 int32 little-endian, 도(degree) x 10,000,000.
 *  SEQ 는 받은 요청의 값을 그대로 되돌려준다(기지국이 SEQ 로 매칭한다).
 *  자율 송신은 하지 않는다. 유효한 POLL 을 받았을 때만 응답한다.
 *  GPS fix 가 없어도 LAT=0 LON=0 VALID=0 인 같은 형식으로 응답한다.
 * ========================================================================== */
#if (TEST_MODE == 6)

#define A4_HDR            0xA4
#define A4_ID_HELMET      0xFD
#define A4_ID_BASE        0xFF
#define A4_TYPE_POLL      0x01
#define A4_TYPE_WAYPOINT  0x02
#define A4_TYPE_ACK       0x03
#define A4_TYPE_UAV_GPS   0x10
#define A4_HDR_LEN        7       /* A4 SRC DST TYPE SEQ_L SEQ_H LEN */
#define A4_ACK_OK         0x00

/* LoRa 페이로드는 최대 255 바이트(RegRxNbBytes 가 uint8)라서 256 으로 잡는다.
   이보다 작게 잡으면 상대가 긴 패킷을 보냈을 때 넘친다. */
static uint8_t  a4_rx[256];

/* ---- 웨이포인트 저장소 -------------------------------------------------- */
#define A4_MAX_WAYPOINTS  15
#define A4_LAT_MIN        (-900000000)
#define A4_LAT_MAX        ( 900000000)
#define A4_LON_MIN        (-1800000000)
#define A4_LON_MAX        ( 1800000000)

typedef struct {
    int32_t lat_1e7;
    int32_t lon_1e7;
} A4_Waypoint_t;

static A4_Waypoint_t wp_list[A4_MAX_WAYPOINTS];
static uint8_t  wp_count;        /* 저장된 웨이포인트 개수 (0~15) */
static uint16_t wp_seq;          /* 마지막으로 저장한 목록의 SEQ */
static bool     wp_new;          /* 새 목록 도착 플래그 */
static bool     wp_have_seq;     /* wp_seq 유효 여부 (중복 판정용) */

static uint32_t a4_rx_cnt;        /* 수신한 전체 LoRa 패킷 수 */
static uint32_t a4_poll_cnt;      /* 유효 POLL 수 */
static uint32_t a4_gps_tx_cnt;    /* GPS 응답 송신 성공 수 */
static uint32_t a4_ack_tx_cnt;    /* ACK 송신 성공 수 */
static uint32_t a4_drop_cnt;      /* 형식이 안 맞아 버린 패킷 수 */
static uint32_t a4_wp_rx_cnt;     /* 새로 저장한 웨이포인트 목록 수 */
static uint32_t a4_wp_dup_cnt;    /* 중복 SEQ 재전송 수 */
static uint16_t a4_last_seq;

/* ---- little-endian 직렬화 ------------------------------------------------ */
static void A4_PutU16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static uint16_t A4_GetU16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void A4_PutI32(uint8_t *p, int32_t v)
{
    uint32_t u = (uint32_t)v;                 /* 음수는 2의 보수 비트 그대로 */
    p[0] = (uint8_t)(u & 0xFFu);
    p[1] = (uint8_t)((u >> 8)  & 0xFFu);
    p[2] = (uint8_t)((u >> 16) & 0xFFu);
    p[3] = (uint8_t)((u >> 24) & 0xFFu);
}

static int32_t A4_GetI32(const uint8_t *p)
{
    uint32_t u = (uint32_t)p[0]
               | ((uint32_t)p[1] << 8)
               | ((uint32_t)p[2] << 16)
               | ((uint32_t)p[3] << 24);
    int32_t out;
    memcpy(&out, &u, sizeof(out));            /* 구현정의 변환 없이 비트 그대로 */
    return out;
}

/* 도(degree) -> 도 x 1e7 int32. 음수 포함 반올림, int32 범위로 클램프.
   위도 ±90 -> ±9e8, 경도 ±180 -> ±1.8e9 로 int32 안에 들어간다. */
static int32_t Deg_To_1e7(double deg)
{
    double v = deg * 10000000.0;
    v = (v >= 0.0) ? (v + 0.5) : (v - 0.5);
    if (v >  2147483647.0) return  2147483647;
    if (v < -2147483648.0) return (int32_t)(-2147483647 - 1);
    return (int32_t)v;
}

/* 도 x 1e7 정수를 "37.1234567" 형태로 (nano.specs 에 %f 가 없어 정수로 조립) */
__attribute__((unused))   /* 좌표를 숫자로 찍고 싶을 때 쓰는 헬퍼 */
static void Fmt_1e7(char *out, size_t n, int32_t v)
{
    const char *sign = "";
    uint32_t a;
    if (v < 0) { sign = "-"; a = (uint32_t)(-(int64_t)v); }
    else       { a = (uint32_t)v; }
    snprintf(out, n, "%s%lu.%07lu",
             sign, (unsigned long)(a / 10000000u), (unsigned long)(a % 10000000u));
}

/* ---- 송신 --------------------------------------------------------------- */

/* A4 FD FF 10 SEQ_L SEQ_H 09 LAT[4] LON[4] VALID   (16 byte) */
static bool A4_SendGpsResponse(uint16_t seq)
{
    uint8_t p[A4_HDR_LEN + 9];
    int32_t lat = 0, lon = 0;
    uint8_t valid = 0;

    if (gps.data.valid) {
        lat   = Deg_To_1e7(gps.data.latitude);
        lon   = Deg_To_1e7(gps.data.longitude);
        valid = 1;
    }

    p[0] = A4_HDR;
    p[1] = A4_ID_HELMET;              /* SRC */
    p[2] = A4_ID_BASE;                /* DST */
    p[3] = A4_TYPE_UAV_GPS;
    A4_PutU16(&p[4], seq);            /* POLL 의 SEQ 를 그대로 */
    p[6] = 9;
    A4_PutI32(&p[7],  lat);
    A4_PutI32(&p[11], lon);
    p[15] = valid;

    bool ok = LoRa_SendPacket(p, (uint8_t)sizeof(p), LORA_TX_TIMEOUT);
    LoRa_RxSet();                     /* 송신 후 STANDBY -> 연속 수신 복귀 */
    return ok;
}

/* 웨이포인트 ACK :  A4 FD FF 03 SEQ_L SEQ_H 02 02 00   (9 byte)
 *   LEN        = 2
 *   PAYLOAD[0] = 0x02  ACK 대상 명령이 WAYPOINT 라는 뜻
 *   PAYLOAD[1] = 0x00  성공
 * 기지국 TTGO 는 웨이포인트 송신 후 약 300ms 만 ACK 를 기다리므로
 * 검증이 끝나면 화면 갱신이나 지연 없이 바로 이 함수를 호출한다.          */
static bool A4_SendWaypointAck(uint16_t seq)
{
    uint8_t ack[9];

    ack[0] = A4_HDR;                       /* 0xA4 */
    ack[1] = A4_ID_HELMET;                 /* SRC = 0xFD */
    ack[2] = A4_ID_BASE;                   /* DST = 0xFF */
    ack[3] = A4_TYPE_ACK;                  /* 0x03 */
    ack[4] = (uint8_t)(seq & 0xFFu);       /* SEQ 는 받은 값 그대로 */
    ack[5] = (uint8_t)((seq >> 8) & 0xFFu);
    ack[6] = 2;                            /* LEN */
    ack[7] = A4_TYPE_WAYPOINT;             /* 0x02 */
    ack[8] = A4_ACK_OK;                    /* 0x00 */

    bool ok = LoRa_SendPacket(ack, (uint8_t)sizeof(ack), LORA_TX_TIMEOUT);
    LoRa_RxSet();                          /* RX Continuous 복귀 */
    return ok;
}

/* ---- 웨이포인트 수신 -----------------------------------------------------
 *  A4 FF FD 02 SEQ_L SEQ_H LEN COUNT  LAT0[4] LON0[4] LAT1[4] LON1[4] ...
 *
 *  좌표에 접근하기 전에 길이/형식 검사를 전부 끝낸다. 어떤 입력이 와도
 *  wp_list 나 pkt 의 범위를 벗어나지 않는다.
 *  검증에 실패하면 기존 목록은 그대로 두고 ACK 도 보내지 않는다.
 * ------------------------------------------------------------------------ */
static void A4_HandleWaypoint(const uint8_t *pkt, uint8_t n)
{
    /* (1) 최소 길이 : COUNT=1 일 때가 16바이트 */
    if (n < 16u)                     { a4_drop_cnt++; return; }
    /* (2)(3)(4)(5) 헤더 / SRC / DST / TYPE */
    if (pkt[0] != A4_HDR)            { a4_drop_cnt++; return; }
    if (pkt[1] != A4_ID_BASE)        { a4_drop_cnt++; return; }
    if (pkt[2] != A4_ID_HELMET)      { a4_drop_cnt++; return; }
    if (pkt[3] != A4_TYPE_WAYPOINT)  { a4_drop_cnt++; return; }

    uint16_t seq   = A4_GetU16(&pkt[4]);
    uint8_t  len   = pkt[6];
    uint8_t  count = pkt[7];

    /* (6) COUNT 범위 */
    if ((count < 1u) || (count > A4_MAX_WAYPOINTS)) { a4_drop_cnt++; return; }
    /* (7) LEN == 1 + COUNT*8   (COUNT<=15 이므로 121 이하, uint8 안에 들어간다) */
    if (len != (uint8_t)(1u + (uint16_t)count * 8u)) { a4_drop_cnt++; return; }
    /* (8) 실제 수신 길이 == 7 + LEN  (= 8 + COUNT*8) 정확히 일치 */
    if ((uint16_t)n != (uint16_t)(7u + (uint16_t)len)) { a4_drop_cnt++; return; }

    /* (9)(10) 좌표 범위. 하나라도 벗어나면 목록 전체를 버린다.
       여기서 읽는 마지막 바이트는 pkt[12 + (count-1)*8 + 3] = pkt[n-1] 이다. */
    for (uint8_t i = 0; i < count; i++) {
        uint16_t off = (uint16_t)8u + (uint16_t)i * 8u;
        int32_t  lat = A4_GetI32(&pkt[off]);
        int32_t  lon = A4_GetI32(&pkt[off + 4u]);
        if ((lat < A4_LAT_MIN) || (lat > A4_LAT_MAX)) { a4_drop_cnt++; return; }
        if ((lon < A4_LON_MIN) || (lon > A4_LON_MAX)) { a4_drop_cnt++; return; }
    }

    a4_last_seq = seq;

    /* 중복 SEQ : ACK 가 유실돼 기지국이 재전송한 경우.
       목록에 덧붙이지 말고 같은 성공 ACK 만 다시 보낸다. */
    if (wp_have_seq && (seq == wp_seq)) {
        a4_wp_dup_cnt++;
        if (A4_SendWaypointAck(seq)) a4_ack_tx_cnt++;
        return;
    }

    /* (11) 검증 통과 -> 기존 목록을 새 목록으로 통째로 교체 */
    for (uint8_t i = 0; i < count; i++) {
        uint16_t off = (uint16_t)8u + (uint16_t)i * 8u;
        wp_list[i].lat_1e7 = A4_GetI32(&pkt[off]);
        wp_list[i].lon_1e7 = A4_GetI32(&pkt[off + 4u]);
    }
    wp_count    = count;
    wp_seq      = seq;
    wp_have_seq = true;
    wp_new      = true;
    a4_wp_rx_cnt++;

    /* 지연 없이 즉시 ACK */
    if (A4_SendWaypointAck(seq)) a4_ack_tx_cnt++;
}

/* ---- 수신 파싱 ----------------------------------------------------------
 *  길이 검사를 가장 먼저 해서 어떤 경우에도 배열 밖을 읽지 않는다.
 *  헤더/SRC/DST/TYPE/LEN 이 하나라도 안 맞으면 조용히 버린다.
 * ------------------------------------------------------------------------ */
static void A4_Handle(const uint8_t *pkt, uint8_t n)
{
    if (n < A4_HDR_LEN)           { a4_drop_cnt++; return; }   /* 헤더도 안 됨 */
    if (pkt[0] != A4_HDR)         { a4_drop_cnt++; return; }
    if (pkt[1] != A4_ID_BASE)     { a4_drop_cnt++; return; }   /* SRC 는 기지국 */
    if (pkt[2] != A4_ID_HELMET)   { a4_drop_cnt++; return; }   /* DST 는 나 */

    uint8_t  type = pkt[3];
    uint16_t seq  = A4_GetU16(&pkt[4]);
    uint8_t  len  = pkt[6];

    /* 선언된 payload 길이만큼 실제로 들어왔는지 확인 */
    if ((uint16_t)n < (uint16_t)A4_HDR_LEN + (uint16_t)len) { a4_drop_cnt++; return; }

    switch (type) {

    case A4_TYPE_POLL:                      /* GPS 요청 */
        if (len != 0) { a4_drop_cnt++; return; }
        a4_poll_cnt++;
        a4_last_seq = seq;
        if (A4_SendGpsResponse(seq)) a4_gps_tx_cnt++;
        break;

    case A4_TYPE_WAYPOINT:                  /* 웨이포인트 수신 (가변 COUNT) */
        A4_HandleWaypoint(pkt, n);          /* 검증 + 저장 + 즉시 ACK */
        break;

    default:                                /* 모르는 TYPE 은 무시 */
        a4_drop_cnt++;
        break;
    }
}

/* ---- 미니맵 --------------------------------------------------------------
 *  [기본 맵]  흰색 선. 아래 PLAZA_* 좌표 3개가 이루는 1/4원(부채꼴) 광장.
 *             수신 데이터와 무관하게 항상 그려지는 고정 배경이다.
 *             세 점 중 "가장 긴 변의 맞은편" 이 부채꼴의 꼭지점이고 나머지
 *             두 점이 호의 양 끝이다.
 *             (실측: 변 95.4 / 97.1 / 134.8 m -> 꼭지점 2번, R 96.3m, 88.9도)
 *
 *  [웨이포인트] GUI 가 보낸 좌표. 작은 파란 점 + 수신 순서대로 파란 직선 연결.
 *  [헬멧]       현재 GPS 위치. 초록 점 하나.
 *
 *  축척은 광장 + 웨이포인트 + 헬멧이 전부 들어가도록 자동. 위쪽이 북쪽.
 * ------------------------------------------------------------------------ */
#define MAP_CX        UI_CX
#define MAP_CY        124
#define MAP_R         84        /* 그림이 들어갈 최대 반지름 */
#define MAP_MARGIN    12        /* 점/번호가 잘리지 않게 남기는 여백 */
#define MAP_TOP       38
#define MAP_BOT       208
#define MAP_ARC_SEG   24
#define MAP_REFRESH_MS 2000

#define MAP_WP_DOT_R   2        /* 웨이포인트 점 반지름 (작게) */
#define MAP_GPS_DOT_R  4        /* 헬멧 점 반지름 */

/* 기본 맵 : 광장 꼭짓점 (도 x 1e7). 다른 장소로 바꾸려면 여기만 고치면 된다. */
static const int32_t PLAZA_LAT[3] = {  373198241,  373204338,  373198090 };
static const int32_t PLAZA_LON[3] = { 1271265125, 1271272694, 1271280355 };

static float Map_Dist(float dx, float dy) { return sqrtf(dx * dx + dy * dy); }

/* 위경도(1e7) -> 기준점 기준 로컬 ENU 미터.
   int32 상태로 먼저 빼기 때문에 float 정밀도 손실이 없다. */
static void Map_ToLocal(int32_t lat, int32_t lon, int32_t rlat, int32_t rlon,
                        float *east_m, float *north_m)
{
    float dlat = (float)(lat - rlat) * 1.0e-7f;
    float dlon = (float)(lon - rlon) * 1.0e-7f;
    float clat = cosf((float)rlat * 1.0e-7f * DEG2RAD);
    *north_m = dlat * 111320.0f;
    *east_m  = dlon * 111320.0f * clat;
}

static void Map_Draw(void)
{
    const int32_t rlat = PLAZA_LAT[0];          /* 투영 기준점 = 광장 1번 꼭짓점 */
    const int32_t rlon = PLAZA_LON[0];

    float   px_[3], py_[3];                     /* 광장 꼭짓점 로컬좌표 */
    float   arcx[MAP_ARC_SEG + 1], arcy[MAP_ARC_SEG + 1];
    float   wex[A4_MAX_WAYPOINTS], wny[A4_MAX_WAYPOINTS];
    float   hx = 0.0f, hy = 0.0f;
    uint8_t nwp = (wp_count > A4_MAX_WAYPOINTS) ? A4_MAX_WAYPOINTS : wp_count;
    bool    hok = gps.data.valid;
    uint8_t apex, e1, e2;

    UI_FillBand(MAP_TOP, MAP_BOT, BLACK);       /* 그림 영역만 지우기 */

    /* ---------- 기본 맵(광장) 기하 ---------- */
    for (uint8_t i = 0; i < 3u; i++) {
        Map_ToLocal(PLAZA_LAT[i], PLAZA_LON[i], rlat, rlon, &px_[i], &py_[i]);
    }
    {
        float d01 = Map_Dist(px_[0] - px_[1], py_[0] - py_[1]);
        float d12 = Map_Dist(px_[1] - px_[2], py_[1] - py_[2]);
        float d20 = Map_Dist(px_[2] - px_[0], py_[2] - py_[0]);

        /* 가장 긴 변(= 호의 현)의 맞은편 꼭짓점이 부채꼴 중심 */
        if      ((d01 >= d12) && (d01 >= d20)) { apex = 2; e1 = 0; e2 = 1; }
        else if ((d12 >= d01) && (d12 >= d20)) { apex = 0; e1 = 1; e2 = 2; }
        else                                   { apex = 1; e1 = 2; e2 = 0; }

        float r1 = Map_Dist(px_[e1] - px_[apex], py_[e1] - py_[apex]);
        float r2 = Map_Dist(px_[e2] - px_[apex], py_[e2] - py_[apex]);
        float rr = 0.5f * (r1 + r2);

        float a1 = atan2f(py_[e1] - py_[apex], px_[e1] - px_[apex]);
        float a2 = atan2f(py_[e2] - py_[apex], px_[e2] - px_[apex]);
        float da = a2 - a1;
        while (da >  3.14159265f) da -= 6.28318531f;   /* 짧은 쪽 호 */
        while (da < -3.14159265f) da += 6.28318531f;

        for (uint8_t k = 0; k <= MAP_ARC_SEG; k++) {
            float a = a1 + da * ((float)k / (float)MAP_ARC_SEG);
            arcx[k] = px_[apex] + rr * cosf(a);
            arcy[k] = py_[apex] + rr * sinf(a);
        }
    }

    /* ---------- 수신 좌표 / 헬멧 ---------- */
    for (uint8_t i = 0; i < nwp; i++) {
        Map_ToLocal(wp_list[i].lat_1e7, wp_list[i].lon_1e7, rlat, rlon, &wex[i], &wny[i]);
    }
    if (hok) {
        Map_ToLocal(Deg_To_1e7(gps.data.latitude), Deg_To_1e7(gps.data.longitude),
                    rlat, rlon, &hx, &hy);
    }

    /* ---------- 자동 축척 : 전부 화면 안에 ---------- */
    float minx = 1.0e30f, maxx = -1.0e30f, miny = 1.0e30f, maxy = -1.0e30f;
    #define MAP_FIT(qx, qy) do {                   \
        if ((qx) < minx) minx = (qx);              \
        if ((qx) > maxx) maxx = (qx);              \
        if ((qy) < miny) miny = (qy);              \
        if ((qy) > maxy) maxy = (qy); } while (0)

    for (uint8_t i = 0; i < 3u; i++)               MAP_FIT(px_[i], py_[i]);
    for (uint8_t k = 0; k <= MAP_ARC_SEG; k++)     MAP_FIT(arcx[k], arcy[k]);
    for (uint8_t i = 0; i < nwp; i++)              MAP_FIT(wex[i], wny[i]);
    if (hok)                                       MAP_FIT(hx, hy);
    #undef MAP_FIT

    float ccx = 0.5f * (minx + maxx);
    float ccy = 0.5f * (miny + maxy);
    float spanx = maxx - minx, spany = maxy - miny;
    float half = 0.5f * ((spanx > spany) ? spanx : spany);
    if (half < 1.0f) half = 1.0f;
    float sc = (float)(MAP_R - MAP_MARGIN) / half;

    #define MAP_SX(e) ((uint16_t)(MAP_CX + (int16_t)(((e) - ccx) * sc)))
    #define MAP_SY(n) ((uint16_t)(MAP_CY - (int16_t)(((n) - ccy) * sc)))

    /* ---------- 1) 기본 맵 : 흰색 선 ---------- */
    {
        uint16_t axp = MAP_SX(px_[apex]), ayp = MAP_SY(py_[apex]);
        ST7789_DrawLine(axp, ayp, MAP_SX(px_[e1]), MAP_SY(py_[e1]), WHITE);
        ST7789_DrawLine(axp, ayp, MAP_SX(px_[e2]), MAP_SY(py_[e2]), WHITE);
        for (uint8_t k = 0; k < MAP_ARC_SEG; k++) {
            ST7789_DrawLine(MAP_SX(arcx[k]),     MAP_SY(arcy[k]),
                            MAP_SX(arcx[k + 1]), MAP_SY(arcy[k + 1]), WHITE);
        }
    }

    /* ---------- 2) 수신 좌표 : 파란 선(순서대로) + 작은 파란 점 ---------- */
    for (uint8_t i = 0; i + 1u < nwp; i++) {
        ST7789_DrawLine(MAP_SX(wex[i]),      MAP_SY(wny[i]),
                        MAP_SX(wex[i + 1u]), MAP_SY(wny[i + 1u]), BLUE);
    }
    for (uint8_t i = 0; i < nwp; i++) {
        uint16_t bx = MAP_SX(wex[i]), by = MAP_SY(wny[i]);
        char num[3];

        ST7789_DrawFilledCircle((int16_t)bx, (int16_t)by, MAP_WP_DOT_R, BLUE);

        /* 순서 번호 (1~15) */
        if ((i + 1u) < 10u) { num[0] = (char)('1' + i); num[1] = '\0'; }
        else                { num[0] = '1'; num[1] = (char)('0' + (i + 1u) - 10u); num[2] = '\0'; }

        int lx = (int)bx + 4, ly = (int)by - 11;
        if (lx > (SCR_W - 16)) lx = (int)bx - 12;      /* 오른쪽이 막히면 왼쪽 */
        if (ly < 2)            ly = (int)by + 4;
        if (lx < 2)            lx = 2;
        if (ly > (SCR_H - 12)) ly = SCR_H - 12;
        ST7789_WriteString((uint16_t)lx, (uint16_t)ly, num, Font_7x10, CYAN, BLACK);
    }

    /* ---------- 3) 헬멧 현재 위치 : 초록 점 하나 ---------- */
    if (hok) {
        ST7789_DrawFilledCircle((int16_t)MAP_SX(hx), (int16_t)MAP_SY(hy),
                                MAP_GPS_DOT_R, GREEN);
    }

    /* ---------- 북쪽 표시 + 축척 ---------- */
    ST7789_WriteString(MAP_CX - 3, MAP_TOP - 1, "N", Font_7x10, GRAY, BLACK);
    {
        char s[12];
        snprintf(s, sizeof(s), "%dm", (int)(half * 2.0f));
        ST7789_WriteString(MAP_CX - 34, MAP_BOT - 10, s, Font_7x10, GRAY, BLACK);
    }
    #undef MAP_SX
    #undef MAP_SY
}

/* ---- 화면 --------------------------------------------------------------- */

static void Mode6_Init(void)
{
    UI_Clear();
    Str_Center(100, "LoRa GPS", Font_11x18, GREEN, BLACK);

    /* GPS 수신 시작 (기존 초기화 그대로) */
    GPS_Init(&gps, &huart1);
    HAL_UART_Receive_IT(&huart1, &gps_rx_byte, 1);

    if (LoRa_Init() != HAL_OK) {
        char s[32];
        snprintf(s, sizeof(s), "VER 0x%02X  FAIL", LoRa_ChipVersion());
        Str_Center(130, s, Font_11x18, RED, BLACK);
        Str_Center(160, "check MISO(PB4)", Font_7x10, GRAY, BLACK);
        return;
    }
    UI_Clear();
}

static void Mode6_Loop(void)
{
    static uint32_t t_ui = 0, t_map = 0;
    static uint32_t map_wp_stamp = 0xFFFFFFFFu;
    static int32_t  map_hlat = 0, map_hlon = 0;
    static bool     map_hok = false;
    uint32_t now = HAL_GetTick();
    char s[40];

    if (!LoRa_IsReady()) { HAL_Delay(200); return; }

    /* GPS NMEA 파싱 (수신은 USART1 인터럽트로 계속 들어옴) */
    GPS_Process(&gps);

    /* LoRa 수신 -> A4 파싱 -> 필요하면 즉시 응답  (지연 금지 구간) */
    uint8_t len = 0, irq = 0;
    int r = LoRa_ReceivePacket(a4_rx, &len, &irq);
    if (r == 1) {
        a4_rx_cnt++;
        A4_Handle(a4_rx, len);
        LoRa_RxSet();               /* 응답을 안 보낸 경우에도 수신 복귀 보장 */
    } else if (r < 0) {
        LoRa_RxSet();               /* CRC 오류 -> 수신 재개 */
    }

    /* ---- 미니맵 : 바뀐 게 있을 때만 다시 그린다 ---- */
    int32_t hlat = gps.data.valid ? Deg_To_1e7(gps.data.latitude)  : 0;
    int32_t hlon = gps.data.valid ? Deg_To_1e7(gps.data.longitude) : 0;
    bool wp_changed  = (a4_wp_rx_cnt != map_wp_stamp);
    bool pos_changed = (gps.data.valid != map_hok) ||
                       (gps.data.valid &&
                        ((hlat - map_hlat > 30) || (map_hlat - hlat > 30) ||
                         (hlon - map_hlon > 30) || (map_hlon - hlon > 30)));

    if (wp_changed || (pos_changed && (now - t_map >= MAP_REFRESH_MS))) {
        map_wp_stamp = a4_wp_rx_cnt;
        map_hlat = hlat; map_hlon = hlon; map_hok = gps.data.valid;
        t_map = now;
        Map_Draw();
    }

    /* ---- 상단 / 하단 상태줄 ---- */
    if (now - t_ui < 400) return;
    t_ui = now;

    if (a4_poll_cnt == 0u) {
        Str_CenterW(16, "WAIT POLL", Font_7x10, YELLOW, BLACK, 15);
    } else {
        snprintf(s, sizeof(s), "WP%u P%lu %ddBm",
                 (unsigned)wp_count, (unsigned long)a4_poll_cnt,
                 (int)LoRa_PacketRssiDbm());
        Str_CenterW(16, s, Font_7x10, GREEN, BLACK, 15);
    }

    /* y=214 는 원형 표시영역의 현이 111px 뿐이라 18자(126px)가 넘친다.
       y=210 / 16자(112px) 로 맞춘다. */
    snprintf(s, sizeof(s), "SAT%02u FIX%u ACK%lu",
             (unsigned)gps.data.satellites, (unsigned)gps.data.fix_quality,
             (unsigned long)((a4_ack_tx_cnt > 99u) ? 99u : a4_ack_tx_cnt));
    Str_CenterW(210, s, Font_7x10, gps.data.valid ? WHITE : GRAY, BLACK, 16);
}
#endif /* TEST_MODE == 6 */

/* ==========================================================================
 *  MODE 7 : 랜덤 좌표 표시 (표시 전용 - LoRa/GPS 안 씀)
 *
 *  2초마다 아래 사각 범위 안에서 위도/경도를 무작위로 뽑아 소수점 7자리로
 *  표시한다. GUI/기지국 연동 없이 화면과 좌표 서식만 확인할 때 쓴다.
 *
 *      위도  37.3187701 ~ 37.3199373   (약 130 m)
 *      경도 127.1260417 ~ 127.1264007  (약  32 m)
 * ========================================================================== */
#if (TEST_MODE == 7)
static uint32_t m7_count;

static void Mode7_Init(void)
{
    UI_Clear();
    Str_Center(30, "POS", Font_11x18, GREEN, BLACK);
    Str_Center(72,  "LAT", Font_7x10, GRAY, BLACK);
    Str_Center(130, "LON", Font_7x10, GRAY, BLACK);
    Str_Center(190, "update 2s", Font_7x10, GRAY, BLACK);
    Rnd_Seed();
}

static void Mode7_Loop(void)
{
    char s[24];

    if (!Rnd_Tick(HAL_GetTick())) { HAL_Delay(20); return; }
    m7_count++;

    Rnd_Fmt(s, sizeof(s), rnd_lat);
    Str_CenterW(88,  s, Font_16x26, WHITE, BLACK, 10);   /* 37.3188315  = 10자 */

    Rnd_Fmt(s, sizeof(s), rnd_lon);
    Str_CenterW(146, s, Font_16x26, WHITE, BLACK, 11);   /* 127.1262044 = 11자 */

    snprintf(s, sizeof(s), "#%lu", (unsigned long)m7_count);
    Str_CenterW(206, s, Font_7x10, GRAY, BLACK, 8);
}
#endif /* TEST_MODE == 7 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* 리셋 원인은 지우기 전에 읽어야 한다. HAL_Init() 보다도 먼저. */
  boot_rccsr = RCC->CSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* --- USART2(레이저) : 보레이트 적용 + 수신 인터럽트 활성화 ---
   *  CubeMX 에서 USART2 global interrupt 를 켜지 않았기 때문에 여기서 직접 켠다.
   *  (stm32f4xx_it.c 의 USART2_IRQHandler 도 함께 추가되어 있음)            */
  huart2.Init.BaudRate = LRF_BAUDRATE;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  /* --- 백라이트 PWM (PB1 / TIM3_CH4) --- */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  BL_Set(BL_BRIGHTNESS);

  /* --- LCD --- */
  ST7789_Init();

  /* --- 스플래시 : 리셋 원인 표시 (이 화면이 반복되면 = MCU 가 리셋 중) --- */
  Fmt_ResetCause(boot_cause, sizeof(boot_cause), boot_rccsr);
  Screen_Splash("IVAS", MODE_NAME);
  HAL_Delay(1500);

  /* --- 선택된 모드 초기화 --- */
#if   (TEST_MODE == 0)
  Mode0_Init();
#elif (TEST_MODE == 1)
  Mode1_Init();
#elif (TEST_MODE == 2)
  Mode2_Init();
#elif (TEST_MODE == 3)
  Mode3_Init();
#elif (TEST_MODE == 4)
  Mode4_Init();
#elif (TEST_MODE == 5)
  Mode5_Init();
#elif (TEST_MODE == 6)
  Mode6_Init();
#elif (TEST_MODE == 7)
  Mode7_Init();
#else
  #error "TEST_MODE 는 0 ~ 7 중 하나여야 합니다."
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if   (TEST_MODE == 0)
    Mode0_Loop();
#elif (TEST_MODE == 1)
    Mode1_Loop();
#elif (TEST_MODE == 2)
    Mode2_Loop();
#elif (TEST_MODE == 3)
    Mode3_Loop();
#elif (TEST_MODE == 4)
    Mode4_Loop();
#elif (TEST_MODE == 5)
    Mode5_Loop();
#elif (TEST_MODE == 6)
    Mode6_Loop();
#elif (TEST_MODE == 7)
    Mode7_Loop();
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|ST7789_DC_Pin|ST7789_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ST7789_RST_Pin|LORA_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  /* LoRa CS 는 반드시 High 로 시작해야 한다. Low 로 두면 디스플레이가 SPI 를
     쓰는 동안 LoRa 가 그 트래픽을 자기 명령으로 받아먹는다. */
  HAL_GPIO_WritePin(GPIOB, LORA_CS_Pin|LORA_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PA1 ST7789_DC_Pin ST7789_CS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|ST7789_DC_Pin|ST7789_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ST7789_RST_Pin LORA_CS_Pin LORA_RST_Pin LORA_EN_Pin */
  GPIO_InitStruct.Pin = ST7789_RST_Pin|LORA_CS_Pin|LORA_RST_Pin|LORA_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_DIO0_Pin */
  GPIO_InitStruct.Pin = LORA_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LORA_DIO0_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        GPS_UART_RxCallback(&gps, gps_rx_byte);
        HAL_UART_Receive_IT(&huart1, &gps_rx_byte, 1);  // 다음 바이트 수신 재시작
    }
    else if (huart->Instance == USART2) {
        LRF_RxCpltCallback(&lrf);                       // 레이저 거리측정 모듈
    }
}

/* 오버런/프레이밍 에러가 나면 수신이 영구히 멈추므로 반드시 재기동 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);

    if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(&huart1, &gps_rx_byte, 1);
    }
    else if (huart->Instance == USART2) {
        LRF_StartRx(&lrf);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
