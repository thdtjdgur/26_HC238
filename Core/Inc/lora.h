/**
  ******************************************************************************
  * @file           : lora.h
  * @brief          : Adafruit RFM9x (SX1276) 900MHz LoRa 드라이버
  ******************************************************************************
  *
  *  ST7789 디스플레이와 SPI1 을 공유한다.
  *    SCK  = PA5   (공유)
  *    MISO = PB4   (SPI1_MISO - LoRa 때문에 새로 추가. 디스플레이는 안 씀)
  *    MOSI = PA7   (공유)
  *    CS   = PB12  (LoRa 전용)
  *    RST  = PB13
  *    G0   = PB14  (DIO0, 입력)
  *    EN   = PB15
  *
  *  두 장치의 SPI 모드가 다르다 :
  *    ST7789 : mode 2 (CPOL=1, CPHA=0), 4 MHz
  *    SX1276 : mode 0 (CPOL=0, CPHA=0), 2 MHz
  *  그래서 LoRa 트랜잭션마다 SPI1->CR1 의 CPOL/CPHA/BR 을 바꿔 끼우고
  *  끝나면 디스플레이 설정으로 되돌린다. (st7789.c 는 수정 불필요)
  *
  *  [무선 파라미터] - ESP32 레퍼런스(lora.c)와 반드시 동일해야 통신이 된다
  *    주파수      922.1 MHz   (FRF = E6 86 66)
  *    대역폭      125 kHz     (MODEM_CONFIG_1 = 0x72, CR 4/5, explicit header)
  *    SF          7           (MODEM_CONFIG_2 = 0x74, payload CRC on)
  *    프리앰블    8 심볼      (기본값, 양쪽 모두 미설정)
  *    싱크워드    0x12        (기본값, 양쪽 모두 미설정)
  *    PA          PA_BOOST    (PA_CONFIG = 0x8F), OCP = 0x31
  *    FIFO        TX base 0x80 / RX base 0x00
  ******************************************************************************
  */

#ifndef LORA_H
#define LORA_H

#include "main.h"
#include <stdbool.h>

/* RegOpMode 값 (bit7 = LongRangeMode/LoRa) - 레퍼런스와 동일 */
#define LORA_WRITE_BIT   0x80
#define LORA_READ_BIT    0x00
#define LORA_SLEEP       0x80
#define LORA_STANDBY     0x81
#define LORA_TX          0x83
#define LORA_RX_CONT     0x85

/* SX1276 이면 RegVersion(0x42) 이 0x12 */
#define LORA_CHIP_VERSION 0x12

/* 초기화 : EN/RST 시퀀스 -> 칩 확인 -> setup/freq/packet/fifo/rx */
HAL_StatusTypeDef LoRa_Init(void);

/* 레퍼런스 lora.c 와 1:1 대응하는 설정 단계들 */
void LoRa_Setup(void);
void LoRa_Freq(void);
void LoRa_PacketSet(void);
void LoRa_FifoSet(void);
void LoRa_RxSet(void);

/* 레지스터 접근 */
void    LoRa_Write(uint8_t reg, uint8_t value);
void    LoRa_WriteBurst(uint8_t reg, const uint8_t *data, uint8_t length);
uint8_t LoRa_Read(uint8_t reg);

/* 송수신 :  1 = 수신됨, 0 = 없음, -1 = CRC 오류 */
int  LoRa_ReceivePacket(uint8_t *buffer, uint8_t *length, uint8_t *irq_flags);
bool LoRa_SendPacket(const uint8_t *buffer, uint8_t length, uint32_t timeout_ms);

/* DIO0(G0) 상승엣지 소프트 검출. EXTI 대신 메인 루프에서 폴링한다. */
bool LoRa_TakeRxFlag(void);
bool LoRa_IrqPending(void);

/* 신호 품질 */
int16_t LoRa_CurrentRssiDbm(void);
int16_t LoRa_PacketRssiDbm(void);
int16_t LoRa_PacketSnrX100(void);

/* 진단용 : 마지막으로 읽은 RegVersion (0x12 여야 정상) */
uint8_t LoRa_ChipVersion(void);
bool    LoRa_IsReady(void);

#endif /* LORA_H */
