/**
  ******************************************************************************
  * @file           : lora.c
  * @brief          : Adafruit RFM9x (SX1276) LoRa 드라이버 - SPI1 을 ST7789 와 공유
  ******************************************************************************
  *  레지스터 설정값은 ESP32 레퍼런스 구현(lora.c)과 완전히 동일하게 유지할 것.
  *  하나라도 다르면 상대 장치와 통신이 안 된다.
  ******************************************************************************
  */

#include "lora.h"
#include "st7789.h"          /* ST7789_CS_* : 버스 충돌 방지용 */
#include <string.h>

/* ---- SX1276 레지스터 (레퍼런스와 동일) ---------------------------------- */
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0B
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS_MASK       0x11
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1A
#define REG_RSSI_VALUE           0x1B
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_PAYLOAD_LENGTH       0x22
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42

#define IRQ_PAYLOAD_CRC_ERROR    0x20
#define IRQ_RX_DONE              0x40
#define IRQ_TX_DONE              0x08

/* ---- SPI 공유 설정 ------------------------------------------------------ */
/* ST7789 : mode 2 (CPOL=1, CPHA=0) @ 16MHz/4 = 4 MHz  (MX_SPI1_Init 기본값) */
#define ST7789_SPI_CPOL          SPI_POLARITY_HIGH
#define ST7789_SPI_CPHA          SPI_PHASE_1EDGE
#define ST7789_SPI_PRESCALER     SPI_BAUDRATEPRESCALER_4
/* SX1276 : mode 0 (CPOL=0, CPHA=0) @ 16MHz/8 = 2 MHz  (레퍼런스도 2 MHz) */
#define LORA_SPI_CPOL            SPI_POLARITY_LOW
#define LORA_SPI_CPHA            SPI_PHASE_1EDGE
#define LORA_SPI_PRESCALER       SPI_BAUDRATEPRESCALER_8
#define LORA_SPI_TIMEOUT         100U

extern SPI_HandleTypeDef hspi1;

static uint8_t lora_tx_buf[257];
static uint8_t lora_rx_buf[257];
static uint8_t lora_version;
static bool    lora_initialized;

/* ========================================================================== */
/*  SPI 버스 중재                                                              */
/* ========================================================================== */

/**
  * @brief  SPI1 의 CPOL/CPHA/보레이트를 바꿔 끼운다.
  * @note   CPOL 변경은 SPE=0 상태에서만 유효하다(RM0368).
  *         디스플레이의 DMA 전송이 끝나기 전에 건드리면 화면이 깨지므로
  *         전송 완료를 먼저 기다린다.
  */
static void LoRa_SpiReconfig(uint32_t cpol, uint32_t cpha, uint32_t presc)
{
    uint32_t t0 = HAL_GetTick();

    while (((hspi1.State != HAL_SPI_STATE_READY) ||
            (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY) != RESET)) &&
           ((HAL_GetTick() - t0) < 100U)) {
        /* 디스플레이 전송 대기 */
    }

    __HAL_SPI_DISABLE(&hspi1);
    MODIFY_REG(hspi1.Instance->CR1,
               SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_BR,
               cpol | cpha | presc);
    hspi1.Init.CLKPolarity       = cpol;
    hspi1.Init.CLKPhase          = cpha;
    hspi1.Init.BaudRatePrescaler = presc;
    __HAL_SPI_ENABLE(&hspi1);

    /* 디스플레이는 송신 전용이라 RX 쪽에 찌꺼기/OVR 가 남아 있다 */
    __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
}

static void LoRa_BusAcquire(void)
{
    /* 디스플레이 CS 를 확실히 High 로 (같은 MOSI/SCK 를 쓰므로 필수) */
    HAL_GPIO_WritePin(ST7789_CS_GPIO_Port, ST7789_CS_Pin, GPIO_PIN_SET);
    LoRa_SpiReconfig(LORA_SPI_CPOL, LORA_SPI_CPHA, LORA_SPI_PRESCALER);
    HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_RESET);
}

static void LoRa_BusRelease(void)
{
    HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_SET);
    /* 디스플레이 설정으로 원복 */
    LoRa_SpiReconfig(ST7789_SPI_CPOL, ST7789_SPI_CPHA, ST7789_SPI_PRESCALER);
}

static void LoRa_Xfer(uint16_t n)
{
    LoRa_BusAcquire();
    HAL_SPI_TransmitReceive(&hspi1, lora_tx_buf, lora_rx_buf, n, LORA_SPI_TIMEOUT);
    LoRa_BusRelease();
}

/* ========================================================================== */
/*  레지스터 접근                                                              */
/* ========================================================================== */

void LoRa_Write(uint8_t reg, uint8_t value)
{
    lora_tx_buf[0] = (uint8_t)(LORA_WRITE_BIT | reg);
    lora_tx_buf[1] = value;
    LoRa_Xfer(2);
}

uint8_t LoRa_Read(uint8_t reg)
{
    lora_tx_buf[0] = (uint8_t)(LORA_READ_BIT | reg);
    lora_tx_buf[1] = 0x00;
    LoRa_Xfer(2);
    return lora_rx_buf[1];
}

void LoRa_WriteBurst(uint8_t reg, const uint8_t *data, uint8_t length)
{
    if (data == NULL || length == 0U) return;

    lora_tx_buf[0] = (uint8_t)(LORA_WRITE_BIT | reg);
    memcpy(&lora_tx_buf[1], data, length);
    LoRa_Xfer((uint16_t)length + 1U);
}

static void LoRa_ReadBurst(uint8_t reg, uint8_t *data, uint8_t length)
{
    if (data == NULL || length == 0U) return;

    memset(lora_tx_buf, 0, (size_t)length + 1U);
    lora_tx_buf[0] = (uint8_t)(LORA_READ_BIT | reg);
    LoRa_Xfer((uint16_t)length + 1U);
    memcpy(data, &lora_rx_buf[1], length);
}

/* ========================================================================== */
/*  설정 단계 - 레퍼런스(lora.c)와 레지스터/값/순서가 완전히 동일해야 한다      */
/* ========================================================================== */

void LoRa_Setup(void)
{
    LoRa_Write(REG_OP_MODE, LORA_SLEEP);
    HAL_Delay(10);
    LoRa_Write(REG_PA_CONFIG, 0x8F);
    LoRa_Write(REG_OCP, 0x31);
    LoRa_Write(REG_IRQ_FLAGS_MASK, 0x00);
    LoRa_Write(REG_IRQ_FLAGS, 0xFF);
}

void LoRa_Freq(void)
{
    const uint8_t frf[3] = { 0xE6, 0x86, 0x66 };   /* 922.1 MHz */

    LoRa_Write(REG_OP_MODE, LORA_SLEEP);
    HAL_Delay(10);
    LoRa_WriteBurst(REG_FRF_MSB, frf, sizeof(frf));
}

void LoRa_PacketSet(void)
{
    LoRa_Write(REG_MODEM_CONFIG_1, 0x72);   /* BW 125 kHz, CR 4/5, explicit header */
    LoRa_Write(REG_MODEM_CONFIG_2, 0x74);   /* SF7, payload CRC on */
}

void LoRa_FifoSet(void)
{
    LoRa_Write(REG_OP_MODE, LORA_STANDBY);
    LoRa_Write(REG_FIFO_TX_BASE_ADDR, 0x80);
    LoRa_Write(REG_FIFO_RX_BASE_ADDR, 0x00);
}

void LoRa_RxSet(void)
{
    LoRa_Write(REG_OP_MODE, LORA_STANDBY);
    LoRa_Write(REG_IRQ_FLAGS, 0xFF);
    LoRa_Write(REG_FIFO_RX_BASE_ADDR, 0x00);
    LoRa_Write(REG_FIFO_ADDR_PTR, 0x00);
    LoRa_Write(REG_DIO_MAPPING_1, 0x00);    /* DIO0 = RxDone */
    LoRa_Write(REG_OP_MODE, LORA_RX_CONT);
}

/* ========================================================================== */

HAL_StatusTypeDef LoRa_Init(void)
{
    if (lora_initialized) return HAL_OK;

    /* GPIO 자체는 MX_GPIO_Init 에서 설정됨. 여기서는 레벨만 잡는다. */
    HAL_GPIO_WritePin(LORA_CS_GPIO_Port,  LORA_CS_Pin,  GPIO_PIN_SET);   /* 비선택 */
    HAL_GPIO_WritePin(LORA_EN_GPIO_Port,  LORA_EN_Pin,  GPIO_PIN_SET);   /* 모듈 ON */
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET); /* 리셋 펄스 */
    HAL_Delay(2);
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    /* 통신 확인 : SX1276 이면 0x12 */
    lora_version = LoRa_Read(REG_VERSION);
    if (lora_version != LORA_CHIP_VERSION) {
        /* 0x00 / 0xFF -> MISO(PB4), CS(PB12), 전원, RST 배선 확인 */
        return HAL_ERROR;
    }

    LoRa_Setup();
    LoRa_Freq();
    LoRa_PacketSet();
    LoRa_FifoSet();
    LoRa_RxSet();

    lora_initialized = true;
    return HAL_OK;
}

int LoRa_ReceivePacket(uint8_t *buffer, uint8_t *length, uint8_t *irq_flags)
{
    uint8_t irq;
    uint8_t received_length;

    if (buffer == NULL || length == NULL || irq_flags == NULL) return -1;

    irq        = LoRa_Read(REG_IRQ_FLAGS);
    *irq_flags = irq;
    *length    = 0;

    if ((irq & IRQ_PAYLOAD_CRC_ERROR) != 0U) {
        LoRa_Write(REG_OP_MODE, LORA_STANDBY);
        LoRa_Write(REG_IRQ_FLAGS, 0xFF);
        return -1;
    }

    if ((irq & IRQ_RX_DONE) == 0U) return 0;

    LoRa_Write(REG_OP_MODE, LORA_STANDBY);
    received_length = LoRa_Read(REG_RX_NB_BYTES);
    LoRa_Write(REG_FIFO_ADDR_PTR, LoRa_Read(REG_FIFO_RX_CURRENT_ADDR));
    LoRa_ReadBurst(REG_FIFO, buffer, received_length);
    *length = received_length;
    LoRa_Write(REG_IRQ_FLAGS, 0xFF);

    return 1;
}

bool LoRa_SendPacket(const uint8_t *buffer, uint8_t length, uint32_t timeout_ms)
{
    uint32_t start;

    if (!lora_initialized || buffer == NULL || length == 0U) return false;

    LoRa_Write(REG_OP_MODE, LORA_STANDBY);
    LoRa_Write(REG_IRQ_FLAGS, 0xFF);
    LoRa_Write(REG_FIFO_ADDR_PTR, 0x80);
    LoRa_WriteBurst(REG_FIFO, buffer, length);
    LoRa_Write(REG_PAYLOAD_LENGTH, length);
    LoRa_Write(REG_DIO_MAPPING_1, 0x40);     /* DIO0 = TxDone */
    LoRa_Write(REG_OP_MODE, LORA_TX);

    start = HAL_GetTick();
    while ((LoRa_Read(REG_IRQ_FLAGS) & IRQ_TX_DONE) == 0U) {
        if ((HAL_GetTick() - start) >= timeout_ms) {
            LoRa_Write(REG_OP_MODE, LORA_STANDBY);
            LoRa_Write(REG_IRQ_FLAGS, 0xFF);
            return false;
        }
        HAL_Delay(1);
    }

    LoRa_Write(REG_OP_MODE, LORA_STANDBY);
    LoRa_Write(REG_IRQ_FLAGS, 0xFF);
    return true;
}

/**
  * @brief  DIO0(G0) 상승엣지 소프트 검출.
  * @note   레퍼런스는 EXTI ISR 로 카운트하지만, 여기서는 메인 루프 폴링으로
  *         같은 역할을 한다. 호출 주기가 패킷 간격보다 짧으면 동작이 동일하다.
  */
bool LoRa_TakeRxFlag(void)
{
    static uint8_t prev = 0;
    uint8_t now = (HAL_GPIO_ReadPin(LORA_DIO0_GPIO_Port, LORA_DIO0_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    bool rising = (now != 0U) && (prev == 0U);
    prev = now;
    return rising;
}

bool LoRa_IrqPending(void)
{
    uint8_t irq = LoRa_Read(REG_IRQ_FLAGS);
    return (irq & (IRQ_RX_DONE | IRQ_PAYLOAD_CRC_ERROR)) != 0U;
}

int16_t LoRa_CurrentRssiDbm(void) { return (int16_t)LoRa_Read(REG_RSSI_VALUE)     - 157; }
int16_t LoRa_PacketRssiDbm(void)  { return (int16_t)LoRa_Read(REG_PKT_RSSI_VALUE) - 157; }

int16_t LoRa_PacketSnrX100(void)
{
    int8_t snr_raw = (int8_t)LoRa_Read(REG_PKT_SNR_VALUE);
    return (int16_t)snr_raw * 25;
}

uint8_t LoRa_ChipVersion(void) { return lora_version; }
bool    LoRa_IsReady(void)     { return lora_initialized; }
