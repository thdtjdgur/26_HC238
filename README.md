# 26_HC238

지상로봇 프로젝트의 펌웨어와 지상국 소프트웨어를 함께 관리한다.

## 구성

- ground_station_robot: 기존 지상로봇 제어 코드
- GUI: 카메라, 지도, 웨이포인트, LoRa/RTK 상태를 표시하는 지상국 GUI
- TTGO: 지상국 TTGO LoRa32 V2.1용 PlatformIO 펌웨어
- N6: NUCLEO-N657X0-Q 사람 객체인식, ESP GPIO 알림 및 배포 파일

## 데이터 흐름

1. N6 카메라가 사람을 70% 이상으로 검출한다.
2. N6 Arduino D2(PD0)가 200 ms 동안 HIGH가 된다.
3. 로봇의 ESP가 상승 에지를 사람 검출 이벤트로 처리한다.
4. GPS와 검출 상태를 LoRa 패킷으로 전송한다.
5. TTGO가 수신한 데이터를 PC GUI로 전달한다.
6. GUI가 검출 위치에 초록색 마커를 표시한다.

각 구성요소의 설치 및 실행 방법은 해당 디렉터리의 README를 참고한다.
