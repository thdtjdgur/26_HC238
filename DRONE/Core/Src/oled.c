#include "oled.h"
#include "oled_font.h"
#include "sensor.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c3;

#define OLED_ADDR 0x78U
#define OLED_PAGE_COUNT 8U
#define OLED_I2C_TIMEOUT_MS 20U
#define OLED_DMA_TIMEOUT_MS 50U

typedef enum
{
    OLED_DMA_IDLE = 0,
    OLED_DMA_CMD,
    OLED_DMA_DATA
} oled_dma_stage_t;

static uint8_t draw_buffer[1024];
static uint8_t frame_buffer[1024];
static uint8_t dma_tx_buffer[160] __attribute__((aligned(32), section(".dma_buffer")));
static volatile uint8_t oled_dma_busy = 0U;
volatile uint8_t oled_update_pending = 0U;
volatile uint8_t oled_dma_page = 0U;
volatile oled_dma_stage_t oled_dma_stage = OLED_DMA_IDLE;
static volatile uint8_t oled_available = 1U;
static volatile uint32_t oled_dma_start_tick = 0U;

static uint32_t OLED_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void OLED_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static HAL_StatusTypeDef OLED_Cmd(uint8_t cmd);
static void OLED_CheckDmaTimeout(void);
static void OLED_StartDmaFrame(void);
static void OLED_StartPageCommand(uint8_t page);
static void OLED_StartPageData(uint8_t page);

uint8_t OLED_IsBusy(void)
{
    OLED_CheckDmaTimeout();
    return oled_dma_busy;
}

static void OLED_AbortDmaFrame(void)
{
    (void)HAL_I2C_Master_Abort_IT(&hi2c3, OLED_ADDR);
    oled_dma_stage = OLED_DMA_IDLE;
    oled_dma_busy = 0U;
    oled_update_pending = 0U;
}

static void OLED_CheckDmaTimeout(void)
{
    if ((oled_dma_busy != 0U) &&
        ((HAL_GetTick() - oled_dma_start_tick) > OLED_DMA_TIMEOUT_MS))
    {
        OLED_AbortDmaFrame();
    }
}

static HAL_StatusTypeDef OLED_Cmd(uint8_t cmd)
{
    uint8_t data[2] = {0x00U, cmd};
    return HAL_I2C_Master_Transmit(&hi2c3, OLED_ADDR, data, 2U, OLED_I2C_TIMEOUT_MS);
}

void OLED_Init(void)
{
    HAL_Delay(100);

    oled_available = 1U;

    if ((OLED_Cmd(0xAE) != HAL_OK) ||
        (OLED_Cmd(0x20) != HAL_OK) ||
        (OLED_Cmd(0x02) != HAL_OK) ||
        (OLED_Cmd(0xB0) != HAL_OK) ||
        (OLED_Cmd(0xC8) != HAL_OK) ||
        (OLED_Cmd(0x00) != HAL_OK) ||
        (OLED_Cmd(0x10) != HAL_OK) ||
        (OLED_Cmd(0x40) != HAL_OK) ||
        (OLED_Cmd(0x81) != HAL_OK) ||
        (OLED_Cmd(0x7F) != HAL_OK) ||
        (OLED_Cmd(0xA1) != HAL_OK) ||
        (OLED_Cmd(0xA6) != HAL_OK) ||
        (OLED_Cmd(0xA8) != HAL_OK) ||
        (OLED_Cmd(0x3F) != HAL_OK) ||
        (OLED_Cmd(0xD3) != HAL_OK) ||
        (OLED_Cmd(0x00) != HAL_OK) ||
        (OLED_Cmd(0xD5) != HAL_OK) ||
        (OLED_Cmd(0x80) != HAL_OK) ||
        (OLED_Cmd(0xD9) != HAL_OK) ||
        (OLED_Cmd(0xF1) != HAL_OK) ||
        (OLED_Cmd(0xDA) != HAL_OK) ||
        (OLED_Cmd(0x12) != HAL_OK) ||
        (OLED_Cmd(0xDB) != HAL_OK) ||
        (OLED_Cmd(0x40) != HAL_OK) ||
        (OLED_Cmd(0x8D) != HAL_OK) ||
        (OLED_Cmd(0x14) != HAL_OK) ||
        (OLED_Cmd(0xAF) != HAL_OK))
    {
        oled_available = 0U;
        return;
    }

    OLED_Clear();
    OLED_Update();

    while (OLED_IsBusy() != 0U)
    {
    }
}

void OLED_Clear(void)
{
    memset(draw_buffer, 0, sizeof(draw_buffer));
}

void OLED_Update(void)
{
    uint32_t primask;

    OLED_CheckDmaTimeout();

    primask = OLED_EnterCritical();

    if (oled_available == 0U)
    {
        OLED_ExitCritical(primask);
        return;
    }

    if (oled_dma_busy != 0U)
    {
        oled_update_pending = 1U;
        OLED_ExitCritical(primask);
        return;
    }

    OLED_ExitCritical(primask);

    OLED_StartDmaFrame();
}

void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
    {
        return;
    }

    if (color != 0U)
    {
        draw_buffer[x + (uint16_t)(y / 8U) * OLED_WIDTH] |= (uint8_t)(1U << (y % 8U));
    }
    else
    {
        draw_buffer[x + (uint16_t)(y / 8U) * OLED_WIDTH] &= (uint8_t)~(1U << (y % 8U));
    }
}

void OLED_Print(uint8_t row, uint8_t col, const char *str)
{
    uint8_t x = (uint8_t)(col * 7U);
    uint8_t y = (uint8_t)(row * 8U);

    while (*str != '\0')
    {
        char c = *str;
        int i;

        if ((c < 32) || (c > 127))
        {
            c = '?';
        }

        for (i = 0; i < 7; i++)
        {
            uint8_t line = font5x7[(uint8_t)c - 32U][i];
            int j;

            for (j = 0; j < 8; j++)
            {
                if ((line & (uint8_t)(1U << (7 - j))) != 0U)
                {
                    OLED_DrawPixel((uint8_t)(x + j), (uint8_t)(y + i), 1U);
                }
            }
        }

        x = (uint8_t)(x + 8U);
        str++;
    }
}

void OLED_Printf(uint8_t row, uint8_t col, const char *fmt, ...)
{
    char buf[64];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    OLED_Print(row, col, buf);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C3)
    {
        return;
    }

    if (oled_dma_stage == OLED_DMA_CMD)
    {
        OLED_StartPageData(oled_dma_page);
        return;
    }

    if (oled_dma_stage == OLED_DMA_DATA)
    {
        uint32_t primask;

        oled_dma_page++;

        if (oled_dma_page < OLED_PAGE_COUNT)
        {
            OLED_StartPageCommand(oled_dma_page);
            return;
        }

        oled_dma_stage = OLED_DMA_IDLE;
        primask = OLED_EnterCritical();
        oled_dma_busy = 0U;

        if (oled_update_pending != 0U)
        {
            oled_update_pending = 0U;
            OLED_ExitCritical(primask);
            OLED_StartDmaFrame();
            return;
        }

        OLED_ExitCritical(primask);
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C3)
    {
        return;
    }

    oled_dma_stage = OLED_DMA_IDLE;
    oled_dma_busy = 0U;
}

static void OLED_StartDmaFrame(void)
{
    uint32_t primask = OLED_EnterCritical();

    if (oled_available == 0U)
    {
        OLED_ExitCritical(primask);
        return;
    }

    memcpy(frame_buffer, draw_buffer, sizeof(frame_buffer));
    oled_update_pending = 0U;
    oled_dma_busy = 1U;
    oled_dma_page = 0U;
    oled_dma_start_tick = HAL_GetTick();
    OLED_ExitCritical(primask);
    OLED_StartPageCommand(0U);
}

static void OLED_StartPageCommand(uint8_t page)
{
    dma_tx_buffer[0] = 0x00U;
    dma_tx_buffer[1] = (uint8_t)(0xB0U + page);
    dma_tx_buffer[2] = 0x00U;
    dma_tx_buffer[3] = 0x10U;
    //SCB_CleanDCache_by_Addr((uint32_t *)dma_tx_buffer, 32);
    oled_dma_stage = OLED_DMA_CMD;

    if (HAL_I2C_Master_Transmit_DMA(&hi2c3, OLED_ADDR, dma_tx_buffer, 4U) != HAL_OK)
    {
        oled_dma_stage = OLED_DMA_IDLE;
        oled_dma_busy = 0U;
        oled_available = 0U;
    }
}

static void OLED_StartPageData(uint8_t page)
{
    dma_tx_buffer[0] = 0x40U;
    memcpy(&dma_tx_buffer[1], &frame_buffer[page * OLED_WIDTH], OLED_WIDTH);
    //SCB_CleanDCache_by_Addr((uint32_t *)dma_tx_buffer, 160);
    oled_dma_stage = OLED_DMA_DATA;

    if (HAL_I2C_Master_Transmit_DMA(&hi2c3, OLED_ADDR, dma_tx_buffer, 129U) != HAL_OK)
    {
        oled_dma_stage = OLED_DMA_IDLE;
        oled_dma_busy = 0U;
        oled_available = 0U;
    }
}
