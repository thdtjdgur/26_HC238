# 지상국 GUI

카메라 영상, UAV/UGV 위치, 웨이포인트, LoRa RSSI 및 사람 검출 위치를 표시한다.

## 개발환경 및 사용 기술

| 구분 | 구성 |
| --- | --- |
| 운영체제 / 언어 | Windows, Python 3.12 (기존 검증: 3.12.9) |
| 화면 구성 | CustomTkinter / Tkinter |
| 지도 | tkintermapview, V-World 지도 타일 또는 기본 지도 |
| 영상 / 이미지 | OpenCV, Pillow, USB UVC 카메라 |
| 그래프 | Matplotlib |
| 통신 | pyserial, USB UART 115200 baud, NTRIP TCP 연결 |
| 의존성 | [requirements.txt](requirements.txt), 최소 버전 범위로 관리 |

## 기능 및 처리 흐름

1. NTRIP 서버에서 받은 RTCM 보정 데이터를 USB 시리얼로 TTGO에 전달합니다.
2. TTGO가 중계한 UAV/UGV 위치와 상태를 해석해 지도 및 관제 화면을 갱신합니다.
3. 수신 텔레메트리에 사람 검출 이벤트가 있으면 전달된 GPS 좌표에 초록색 발견 마커를 표시합니다.
4. 지도에서 설정한 웨이포인트를 패킷으로 만들어 TTGO에 전달합니다.
5. USB 카메라 영상과 LoRa RSSI 상태를 함께 표시합니다.

사람 검출은 N6가 수행하며, GUI는 수신된 검출 정보와 위치를 표시합니다. 발견 마커의 좌표는 전달된 GPS 좌표이고 영상 픽셀만으로 사람의 절대 위치를 계산하지 않습니다.

## 연결 및 설정 확인

- PC ↔ TTGO: USB UART. 실제 장치 관리자 포트에 맞춰 `GCS_TTGO_PORT`를 설정합니다.
- PC ↔ N6: 영상 확인에는 N6의 UVC USB 연결과 `CONFIG["CAM_INDEX"]` 선택이 필요합니다.
- NTRIP: 호스트, 포트, mountpoint, 계정 및 비밀번호를 환경변수로 설정합니다.
- [.env.example](.env.example)은 설정 목록을 보여주는 템플릿입니다. 현재 코드는 이 파일을 자동으로 읽지 않으므로 PowerShell 환경변수로 설정합니다.
- 통신 패킷 정의는 [ground_protocol.py](ground_protocol.py), 통신 주기 관리는 [keytest.py](keytest.py)를 확인합니다.

## 요구사항

- Windows
- Python 3.12 권장
- TTGO 기본 포트: COM8
- N6 UVC 카메라 또는 노트북 카메라

## 설치

    py -3.12 -m venv .venv
    .\.venv\Scripts\activate
    python -m pip install -r requirements.txt

## 환경변수

.env.example을 참고해 PowerShell 세션에 필요한 값을 설정한다. 실제 키와 계정은 저장소에 커밋하지 않는다.

    $env:VWORLD_API_KEY="발급받은_VWORLD_키"
    $env:GCS_TTGO_PORT="COM8"
    $env:NTRIP_USER="NTRIP_계정"
    $env:NTRIP_PASSWORD="NTRIP_비밀번호"

VWORLD_API_KEY가 없으면 tkintermapview의 기본 지도 타일을 사용한다. run.ps1은 한국어 Windows 콘솔에서 이모지 출력 오류가 나지 않도록 UTF-8을 설정한다.

## 실행

    .\run.ps1

기본 카메라 번호는 guitest.py의 CONFIG 안 CAM_INDEX에서 변경한다.

## 테스트

    python -m unittest test_keytest_round_robin.py

## 주요 파일

- guitest.py: GUI 진입점
- keytest.py: NTRIP, TTGO 시리얼, 반이중 주기 관리
- ground_protocol.py: UAV/UGV 메시지 인코딩 및 디코딩
- DRONE.png, ugv.png, TOWER.png: 지도 아이콘
