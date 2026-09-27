# RTKTTGO 지상국 LoRa 펌웨어

이 프로젝트는 노트북의 `KEYTEST`와 지상국 TTGO LoRa32 V2.1 사이를 연결한다.

전체 반이중 통신 cycle은 3초이며, 한 cycle 안에서 RTCM 송신, UGV 응답,
헬멧 GPS poll/응답 및 선택적인 Waypoint 송신을 순서대로 처리한다.

## 동작 순서

1. COM8, 115200 baud로 PC 패킷을 받는다.
2. `FF F4`의 RTCM 묶음을 완전한 RTCM3 프레임으로 분리하고 CRC24Q를 검사한다.
3. RTCM 프레임이 126바이트 이하면 `A1`, 더 크면 `A2` 조각들로 LoRa 송신한다.
4. `FF F2` Waypoint는 기존 패킷을 바꾸지 않고 그대로 LoRa 송신한다.
5. `FF F5`를 받으면 LoRa RX Continuous 모드로 전환한다.
6. 드론 패킷은 GUI용 `P,...` 라인, 배경 RSSI는 `R,...` 라인으로 PC에 전달한다.

## OLED 표시

- RTCM TX: 메시지 타입, SEQ, 원본 프레임 길이, fragment 번호, 성공 여부
- Waypoint TX: Waypoint 개수, 패킷 바이트 수, 성공 여부
- RX 대기: LoRa RX 모드와 드론 응답 대기 상태
- 드론 RX: 위도, 경도, 사람 검출 여부, 검출 인원 수
- 수신 오류: LoRa payload CRC 오류
- UAV GPS 정상 수신: `UAV RX OK`, SEQ, Fix 상태, 위도, 경도

UAV GPS 응답 및 Waypoint ACK 대기 시간은 300 ms이며, UGV 응답 대기 시간은
기존 180 ms를 유지한다.

## PC에서 받는 패킷

```text
RTCM     : FF F4 LEN_L LEN_H [RTCM3 프레임들]
Waypoint : FF F2 COUNT [LAT int32 LE][LON int32 LE]...
RX 전환  : FF F5
```

## LoRa로 보내는 RTCM 패킷

```text
A1 SEQ [완전한 RTCM3 프레임]
A2 SEQ INDEX COUNT [RTCM 조각]
```

기존 TX FIFO 시작 주소가 `0x80`이므로 한 LoRa payload를 최대 128바이트로 제한한다.
RTCM 원본의 어떤 바이트도 임의로 버리지 않는다.

## 빌드 및 업로드

GUI가 COM8을 사용 중이면 먼저 종료한 뒤 업로드해야 한다.

```powershell
platformio run
platformio run --target upload
```
