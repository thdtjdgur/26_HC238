#pragma once
// 헤더파일 안에 선언해서 이 헤더파일 혹시 다른 파일에서 또 접근했으면 두번 접근해서 똑같은 내용 또 쓰고있는거니깐 나가라 하고 중복 접근을 자동으로 막아주는거
//그게 아니라 클래식한 방법으로 수동으로 할라면 ifndef활용해서 한번만 들오게 구성(요즘은 다 pragma로 함)

#include <Arduino.h>

void oled_init(void);
// 아두이노 프로그래밍에서 파일 전체에서 setup과 loop함수는 딱 한개만 실행됨 그리고 main문 먼저 드가니깐 거의 main의 setup과 loop가 실행될것
// 이렇게 파일을 분할해서 쓸때 main이 아닌 다른 파일의 setup과 loop는 사실 의미가 없어지며, 이렇게 다른 파일의 setup에 들어갈 내용은 별도로 이렇게 초기화 함수를 파서
// 이렇게 전달해줘야함(setup,loop를 넘길수가 없을뿐 아니라, 넘겨도 이름 다 같아서 어짜피 못쓴다)

void oled_show_rx_waiting(void);
void oled_show_rx_crc_error(void);

void oled_update_rtcm_tx(uint16_t rtcm_type, uint8_t seq, uint16_t frame_len,
                         uint8_t fragment_index, uint8_t fragment_count,
                         int8_t tx_status);

void oled_update_waypoint_tx(uint8_t waypoint_count, uint8_t packet_length,
                             int8_t tx_status);

void oled_update_gps_packet(const uint8_t *rx_buf, uint8_t len);
void oled_update_uav_packet(const uint8_t *rx_buf, uint8_t len);
