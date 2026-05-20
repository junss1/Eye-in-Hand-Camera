# 경계 시스템 자동화

[![Demo Video](docs/eye_in_hand.png)](https://youtu.be/Xig3LvbgLYU)
↑ 이미지 클릭 시 데모 영상을 확인할 수 있습니다.

YOLO 기반 영상 인식 결과와 ROS 2 토픽/액션 흐름을 이용해 협동로봇의 추적, 인증, 후속 동작을 제어하는 프로젝트입니다.

이 프로젝트는 카메라 영상에서 대상의 위치를 검출하고, 화면 중심 대비 정규화 오차(`error_norm`)를 계산한 뒤 Doosan 로봇의 TCP 속도 명령(`speedl`)에 반영합니다. 인증 결과에 따라 경례 또는 사격 동작을 실행합니다.

---

## 1. 프로젝트 개요

- Arduino 기반 외부 액추에이터 제어 추가
- 저장소 구조 및 Hardware Configuration 반영 완료

---

## 3. 저장소 구조

```text
Eye-in-Hand-Camera/
├── README.md
├── LICENSE
├── requirements.txt
├── .gitignore
├── arduino/
│   └── sg90_controller/
│       └── sg90_controller.ino
```

### 폴더 설명

| 경로 | 설명 |
|---|---|
| `arduino/sg90_controller/` | Arduino 기반 SG90 액추에이터 제어 코드 |

---

## 12. Arduino External Actuator

외부 액추에이터 제어를 위해 Arduino 기반 SG90 서보모터 제어 코드를 사용합니다.

### 저장 위치 예시

```text
arduino/
└── sg90_controller/
    └── sg90_controller.ino
```

### Hardware

| Item | Value |
|---|---|
| Board | Arduino Uno / Nano |
| Servo | SG90 |
| Signal Pin | D9 |
| Baudrate | 115200 |

### Serial Command

| Command | 설명 |
|---|---|
| `1` | 서보모터 왕복 동작 실행 |

### 동작 개요

```text
ROS 2 shoot node
→ Serial Communication
→ Arduino
→ SG90 Servo Actuation
```

---

## 13. Hardware Configuration

| 항목 | 내용 |
|---|---|
| External Device | Arduino 기반 SG90 서보 액추에이터 (`/dev/ttyACM0`) |
