# 지상국 GUI

카메라 영상, UAV/UGV 위치, 웨이포인트, LoRa RSSI 및 사람 검출 위치를 표시한다.

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
