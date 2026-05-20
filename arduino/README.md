## Arduino External Actuator

SG90 서보모터를 Arduino Serial 통신으로 제어합니다.

### Hardware

| Item | Value |
|---|---|
| Board | Arduino Uno / Nano |
| Servo | SG90 |
| Signal Pin | D9 |
| Baudrate | 115200 |

### Serial Command

| Command | Description |
|---|---|
| `1` | 서보모터를 지정 각도로 3회 왕복 동작 |
| `HOME` | 서보모터를 기본 위치로 복귀 |

### Default Parameters

| Parameter | Value |
|---|---|
| `HOME_ANGLE` | 90 |
| `STEP_DEG` | 40 |
| `HOLD_MS` | 200 |
| `REPEAT_CNT` | 3 |