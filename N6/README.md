# STM32N6 사람 객체인식

NUCLEO-N657X0-Q와 카메라에서 YOLOv8n person 모델을 실행하고, 한 프레임이라도 사람 신뢰도가 70% 이상이면 Arduino D2(PD0)에 200 ms HIGH 펄스를 출력한다.

사람이 계속 검출되는 동안에는 펄스를 반복하지 않는다. 검출이 사라진 뒤 다시 나타나면 새 펄스를 출력한다.

## 개발환경 및 사용 기술

| 구분 | 구성 |
| --- | --- |
| 대상 보드 | NUCLEO-N657X0-Q, STM32N657 / Cortex-M55 및 Neural-ART NPU |
| 언어 | C, STM32 HAL / ST BSP |
| 추론 모델 | YOLOv8n, 320×320 입력, 양자화 TFLite, person 단일 클래스 |
| 모델 변환 | STM32AI Model Zoo Services, STEdgeAI 4.0 / x-cube-ai pack 12.0.0 |
| 빌드 | STM32CubeIDE 1.17.0, GNU Tools for STM32 12.3.1 |
| PC 도구 | Python 3.12.9, STM32CubeProgrammer |
| 영상 출력 | UVCL 구성 / USB UVC |
| 이벤트 출력 | Arduino D2(PD0), 3.3 V GPIO |

## 주요 기능 및 처리 흐름

1. ST 카메라 파이프라인에서 영상을 받아 모델 입력으로 전처리합니다.
2. Neural-ART NPU에서 사람 객체 인식을 실행하고 후처리 결과를 얻습니다.
3. 후처리 결과 중 신뢰도 `>= 0.70`인 검출이 한 번이라도 있으면 GPIO HIGH 출력을 시작합니다. 연속 프레임 확인을 기다리지 않습니다.
4. GPIO는 추론 루프에서 200 ms 경과를 확인한 뒤 LOW로 복귀하므로 실제 펄스 길이는 루프 처리 시간에 따라 더 길 수 있습니다.
5. ESP가 GPIO 이벤트를 감지해 GPS와 검출 플래그를 LoRa로 전달하고, 지상국 GUI가 수신 좌표를 표시합니다.

N6 코드는 사람 검출과 GPIO 출력까지 담당합니다. ESP의 검출 플래그 저장, GPS 결합 및 LoRa 패킷 송신은 로봇 측 코드의 담당 기능입니다.

## 재현 범위

이 디렉터리는 ST 프로젝트 전체 복사본이 아니라 변경 소스, 설정, upstream 패치와 플래시용 펌웨어를 제공합니다. 바로 실행하려면 아래 플래시 절차를 사용하고, 다시 빌드하려면 아래에 명시한 ST 저장소 커밋과 도구 버전에 변경 파일을 적용합니다.

## 배선

- N6 Arduino D2(PD0) -> 로봇 ESP 입력 GPIO
- N6 GND -> ESP GND
- GPIO 전압: 3.3 V
- ESP 입력은 상승 에지 인터럽트와 풀다운 입력을 권장한다.

## 바로 플래시하기

필수 도구:

- STM32CubeProgrammer
- NUCLEO-N657X0-Q ST-LINK USB 연결
- N6 외장 Flash용 MX25UM51245G external loader

절차:

1. 보드를 DEV BOOT로 설정한다.
2. PowerShell에서 다음 명령을 실행한다.

       .\scripts\flash.ps1

3. 보드를 FLASH BOOT로 변경한다.
4. ST-LINK와 CN8 USB를 모두 분리한 뒤 다시 연결한다.
5. Windows 카메라 앱에서 STM32 UVC를 선택한다.

flash.ps1은 아래 주소에 기록한다.

- ai_fsbl.hex: HEX 파일에 주소 포함
- NUCLEO-N657X0-Q_GettingStarted_ObjectDetection_signed.bin: 0x70100000
- network_atonbuf.xSPI2.bin: 0x70380000

## 소스에서 재생성하기

기준 저장소:

- stm32ai-modelzoo-services commit: 0f6210ed5156126b782e1c43249063a477484b20
- STM32N6-GettingStarted-ObjectDetection commit: 7ae96b5452183664c0d9b3dfe06a82a6ed0e59cb

필요 도구:

- Python 3.12.9
- STM32CubeIDE 1.17.0
- GNU Tools for STM32 12.3.1
- STEdgeAI 4.0 / x-cube-ai pack 12.0.0
- STM32CubeProgrammer

적용 파일:

- src/main.c를 ST ObjectDetection 프로젝트의 Application/NUCLEO-N657X0-Q/Src/main.c에 복사
- config/stmaic_NUCLEO-N657X0-Q.conf를 ObjectDetection 프로젝트 루트에 복사
- config/deployment_n6_yolov8n320_person.yaml을 modelzoo-services/object_detection/config_file_examples에 복사

원본 모델:

https://github.com/stm32-hotspot/ultralytics/blob/main/examples/YOLOv8-STEdgeAI/stedgeai_models/object_detection/yolov8n_320_quant_pc_uf_od_coco-person.tflite

모델 SHA-256:

14214FDE0AF656E24FA784DE28337C1CCF459E63C1D19BCA2BF9CC47ABD63537

YAML의 model_path, path_to_stedgeai, path_to_cubeIDE는 각 PC 설치 경로에 맞춰 수정한다. stmaic 설정의 서명 명령에 있는 -hv 2.3 -align 옵션은 FLASH BOOT에 필요하므로 제거하지 않는다.

배포 명령:

    cd object_detection
    python -u stm32ai_main.py --config-path ./config_file_examples/ --config-name deployment_n6_yolov8n320_person.yaml

## 파일 설명

- src/main.c: 70% 판정 및 D2(PD0) 펄스 구현
- config/deployment_n6_yolov8n320_person.yaml: 모델과 후처리 임계값
- config/stmaic_NUCLEO-N657X0-Q.conf: 빌드, 서명, 플래시 설정
- patches/n6-person-gpio.patch: 기준 ST 프로젝트에 대한 변경사항
- firmware: 검증된 FSBL, 애플리케이션, NPU 모델 데이터
- licenses/ST-LICENSE.md: ST 구성요소 라이선스 고지
