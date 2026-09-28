#pragma once
#include <Arduino.h>
#include <SPI.h>


#define MOSI 27
#define MISO 19
#define SCLK 5
#define CS 18
#define DIO 26
#define RST 23

#define write_bit 0x80
#define read_bit 0x00

#define sleep 0x80 // 레지스터 설정중에서 LORA모드 변경 + SLEEP 상태(SPI만됨, RF정지)에서 설정할수있는 모드,레지스터가있음,저전력모드. 즉, 시동을 끄고 좀 설정하는 느낌
                   // 전원을 처음켜면 FSK/OSK모드여서 이걸 LORA모드로 사용하기 위해서는 무조건 SLEEP상태에서만 가능(stanby불가능) 그래서 전원 꼽고 이거 한번 켜주고 레지스터 설정해야 lora모드의 레지스터의 주소로 값이 들어감
#define stanby 0x81 // RF 송수신은 아직 안 하지만, 클럭 켜지고, 전력좀 쓰고. 즉, 시동키고 대기하는 느낌
#define tx 0x83

#define no_more 0xFF // burst accecss에서 2개이상으로 하고싶지는 않을때


void lora_setup(void);

//lora 기능관련 함수
void lora_write(uint8_t adr, uint8_t data);
void lora_write_burst(uint8_t adr, uint8_t data1, uint8_t data2, uint8_t data3);
uint8_t lora_read(uint8_t adr);
void lora_freq(void);
void packet_set(void);

//tx관련
void fifo_set(void);
void payload_write(const uint8_t *buf, uint8_t len);
bool real_tx(uint32_t timeout_ms = 2000);

//rx관련
void rx_set(void);
void rx_read(void);
int16_t lora_current_rssi_dbm(void);
int16_t lora_packet_rssi_dbm(void);
int16_t lora_packet_snr_x100(void);
