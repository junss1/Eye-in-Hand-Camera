#include <Servo.h>

Servo sg90;

const int SERVO_PIN = 9;

const int HOME_ANGLE = 90;   // 기본 위치
const int STEP_DEG   = 40;   // 이동 각도
const int HOLD_MS    = 200;  // 각 위치 유지 시간(ms)
const int REPEAT_CNT = 3;    // 왕복 횟수

void setup() {
  sg90.attach(SERVO_PIN);
  sg90.write(HOME_ANGLE);
  delay(300);

  Serial.begin(115200);
  Serial.setTimeout(30);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "1") {
      moveServo();
      Serial.println("DONE");
    }
    else if (command == "HOME") {
      sg90.write(HOME_ANGLE);
      Serial.println("HOME");
    }
  }
}

void moveServo() {
  int target = HOME_ANGLE - STEP_DEG;

  if (target < 0) {
    target = 0;
  }

  for (int i = 0; i < REPEAT_CNT; i++) {
    sg90.write(target);
    delay(HOLD_MS);

    sg90.write(HOME_ANGLE);
    delay(HOLD_MS);
  }
}