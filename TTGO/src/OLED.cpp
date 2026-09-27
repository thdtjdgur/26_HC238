#include <Arduino.h>
#include <Wire.h>  // OLED가 I2C로 연결되있어서 I2C연결위한 I2C통신 라이브러리
#include "SSD1306Wire.h" //SSD1306C 칩이 달린 OLED에 글자 찍을수있게 해주는 라이브러리

#include "oled.h"

#define column0 0

// TTGO LoRa32 v2.1.x (보통 OLED I2C: SDA=21, SCL=22, addr=0x3C)
static const int SDA_PIN = 21;
static const int SCL_PIN = 22;

SSD1306Wire display(0x3C, SDA_PIN, SCL_PIN);
// 여기서 디스플레시의 주소 0x3C, SDA,SCL어느핀인지 전달 // SSD1306Wire라는 C++에서 CLASS에 display라는 인스턴스(구조체 변수 감성)
// 아래에서 전부다 이제 이 인스턴스에 접근해서 전달된 인자값으로 막 하는중

static void draw_title(const char *title)
{
    display.clear();
    display.setColor(WHITE);
    display.setFont(ArialMT_Plain_10); // 글자 폰트 + 크기(10,16,24만 가능)
    display.drawString(column0, 0, title);
    display.drawHorizontalLine(0, 12, 128);
}

void oled_init(void)
{
    display.init();
    display.flipScreenVertically(); // 디스플레이가 거꾸로 배치되있어서 그거 반대로 돌리는거(우리꺼는 방향이 반대로 되있음)
    draw_title("<RTK GROUND TTGO>");
    display.drawString(column0, 20, "LoRa RX WAITING");
    display.display(); // 실제 디스플레이에 띄우기
}

void oled_show_rx_waiting(void)
{
    draw_title("RX MODE");
    display.drawString(column0, 20, "Waiting Drone...");
    display.display();
}

void oled_show_rx_crc_error(void)
{
    draw_title("RX ERROR");
    display.drawString(column0, 20, "LoRa CRC ERROR");
    display.display();
}

void oled_update_rtcm_tx(uint16_t rtcm_type, uint8_t seq, uint16_t frame_len,
                         uint8_t fragment_index, uint8_t fragment_count,
                         int8_t tx_status)
{
    char line1[24];
    char line2[24];
    char line3[24];
    const char *status = tx_status == 0 ? "SENDING" :
                         (tx_status > 0 ? "TX OK" : "TX ERR");

    snprintf(line1, sizeof(line1), "TYPE:%u SEQ:%u", rtcm_type, seq);
    snprintf(line2, sizeof(line2), "FRAME:%u B", frame_len);
    if(fragment_count > 1)
    {
        snprintf(line3, sizeof(line3), "%s %u/%u", status,
                 (uint8_t)(fragment_index + 1), fragment_count);
    }
    else
    {
        snprintf(line3, sizeof(line3), "%s SINGLE", status);
    }

    draw_title("TX RTCM");
    display.drawString(column0, 16, line1);
    display.drawString(column0, 30, line2);
    display.drawString(column0, 44, line3);
    display.display();
}

void oled_update_waypoint_tx(uint8_t waypoint_count, uint8_t packet_length,
                             int8_t tx_status)
{
    char line1[24];
    char line2[24];

    snprintf(line1, sizeof(line1), "COUNT:%u", waypoint_count);
    const char *status = tx_status == 0 ? "SENDING" :
                         (tx_status > 0 ? "TX OK" : "TX ERR");
    snprintf(line2, sizeof(line2), "BYTES:%u %s", packet_length, status);

    draw_title("TX WAYPOINT");
    display.drawString(column0, 20, line1);
    display.drawString(column0, 36, line2);
    display.display();
}

void oled_update_gps_packet(const uint8_t *rx_buf, uint8_t len)
{
    // 드론 GPS 패킷 형식:
    // [0] id   = 0xFE
    // [1] seq
    // [2] comp = 0xF3
    // [3] len  = 1
    // [4~7]  lat int32 little-endian
    // [8~11] lon int32 little-endian
    // 그 뒤 [detected][person_count]가 이어진다.
    if(rx_buf == nullptr || len < 14) return;
    if(rx_buf[0] != 0xFE || rx_buf[2] != 0xF3) return;

    const uint8_t gps_count = rx_buf[3];
    if(gps_count == 0) return;

    const uint16_t n6_offset = (uint16_t)(4 + gps_count * 8);
    if((uint16_t)len < n6_offset + 2) return;

    int32_t lat_i = (int32_t)(
        (uint32_t)rx_buf[4] |
        ((uint32_t)rx_buf[5] << 8) |
        ((uint32_t)rx_buf[6] << 16) |
        ((uint32_t)rx_buf[7] << 24)
    );

    int32_t lon_i = (int32_t)(
        (uint32_t)rx_buf[8] |
        ((uint32_t)rx_buf[9] << 8) |
        ((uint32_t)rx_buf[10] << 16) |
        ((uint32_t)rx_buf[11] << 24)
    );

    const double lat = lat_i / 10000000.0;
    const double lon = lon_i / 10000000.0;
    const uint8_t detected = rx_buf[n6_offset];
    const uint8_t person_count = rx_buf[n6_offset + 1];

    char lat_buf[24];
    char lon_buf[24];
    char person_buf[24];

    snprintf(lat_buf, sizeof(lat_buf), "LAT:%.6f", lat);
    snprintf(lon_buf, sizeof(lon_buf), "LON:%.6f", lon);
    snprintf(person_buf, sizeof(person_buf), "PERSON:%s CNT:%u",
             detected ? "YES" : "NO", person_count);

    draw_title("RX DRONE");
    display.drawString(column0, 15, lat_buf);
    display.drawString(column0, 29, lon_buf);
    display.drawString(column0, 43, person_buf);
    display.display(); // 실제 디스플레이에 띄우기
}

void oled_update_uav_packet(const uint8_t *rx_buf, uint8_t len)
{
    if(rx_buf == nullptr || len < 15) return;
    if(rx_buf[0] != 0xA4 || rx_buf[1] != 0xFD || rx_buf[2] != 0xFF) return;
    if(rx_buf[3] != 0x10 || (rx_buf[6] != 8 && rx_buf[6] != 9)) return;
    if(len != (uint8_t)(7 + rx_buf[6])) return;

    const uint16_t seq = (uint16_t)rx_buf[4] | ((uint16_t)rx_buf[5] << 8);
    const bool valid = (rx_buf[6] == 8) ? true : (rx_buf[15] == 1);
    int32_t lat_i = (int32_t)(
        (uint32_t)rx_buf[7] |
        ((uint32_t)rx_buf[8] << 8) |
        ((uint32_t)rx_buf[9] << 16) |
        ((uint32_t)rx_buf[10] << 24)
    );
    int32_t lon_i = (int32_t)(
        (uint32_t)rx_buf[11] |
        ((uint32_t)rx_buf[12] << 8) |
        ((uint32_t)rx_buf[13] << 16) |
        ((uint32_t)rx_buf[14] << 24)
    );

    char status_buf[24];
    char lat_buf[24];
    char lon_buf[24];
    snprintf(status_buf, sizeof(status_buf), "SEQ:%u GPS:%s", seq,
             valid ? "VALID" : "NO FIX");
    snprintf(lat_buf, sizeof(lat_buf), "LAT:%.6f", lat_i / 10000000.0);
    snprintf(lon_buf, sizeof(lon_buf), "LON:%.6f", lon_i / 10000000.0);

    draw_title("UAV RX OK");
    display.drawString(column0, 15, status_buf);
    display.drawString(column0, 29, lat_buf);
    display.drawString(column0, 43, lon_buf);
    display.display();
}
