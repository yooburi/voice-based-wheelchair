#include <Arduino.h>

// ==== 핀 설정 ====
const uint8_t SPD1  = 6;   // 모터1 PWM 출력
const uint8_t RUN1  = 23;  // 모터1 Run enable
const uint8_t RVS1  = 25;  // 모터1 방향
const uint8_t PLSO1 = 2;   // 모터1 속도 피드백 (INT0)

const uint8_t SPD2  = 7;   // 모터2 PWM 출력
const uint8_t RUN2  = 22;  // 모터2 Run enable
const uint8_t RVS2  = 24;  // 모터2 방향
const uint8_t PLSO2 = 3;   // 모터2 속도 피드백 (INT1)

// ==== 카운터 ====
volatile uint32_t plsoCount1 = 0;
volatile uint32_t plsoCount2 = 0;
void isrPlso1() { plsoCount1++; }
void isrPlso2() { plsoCount2++; }

// ==== PID 파라미터 ====
float Kp = 0.8, Ki = 0.2, Kd = 0.0;
float set_rps1 = 150.0;   // 목표속도 [회전/초] (양수=정방향, 음수=역방향)
float set_rps2 = -150.0;  // 두 번째 모터는 반대방향 예시

// ==== 내부 상태 ====
uint32_t lastCount1 = 0, lastCount2 = 0;
uint32_t lastTime = 0;
float integral1 = 0, integral2 = 0;
float lastError1 = 0, lastError2 = 0;
float duty1 = 0, duty2 = 0;

void setup() {
  pinMode(SPD1, OUTPUT);
  pinMode(RUN1, OUTPUT);
  pinMode(RVS1, OUTPUT);
  pinMode(PLSO1, INPUT_PULLUP);

  pinMode(SPD2, OUTPUT);
  pinMode(RUN2, OUTPUT);
  pinMode(RVS2, OUTPUT);
  pinMode(PLSO2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PLSO1), isrPlso1, RISING);
  attachInterrupt(digitalPinToInterrupt(PLSO2), isrPlso2, RISING);

  // RUN_ACTIVE_LOW 가정 → LOW=Run
  digitalWrite(RUN1, LOW);
  digitalWrite(RUN2, LOW);

  Serial.begin(115200);
}

void loop() {
  uint32_t now = millis();
  if (now - lastTime >= 100) {   // 100ms마다 속도 계산
    // === 모터1 ===
    uint32_t cur1 = plsoCount1;
    uint32_t delta1 = cur1 - lastCount1;
    lastCount1 = cur1;

    float rps1 = (delta1 / 6.0) / 0.1;  // 6펄스=1회전 가정

    float error1 = abs(set_rps1) - rps1;   // 속도 오차 (절댓값 기준)
    integral1 += error1 * 0.1;
    float deriv1 = (error1 - lastError1) / 0.1;
    lastError1 = error1;

    float output1 = Kp*error1 + Ki*integral1 + Kd*deriv1;
    duty1 += output1;
    duty1 = constrain(duty1, 0, 255);

    // 방향 제어: set_rps1 부호 기준
    if (set_rps1 >= 0) digitalWrite(RVS1, HIGH);  // 정방향
    else               digitalWrite(RVS1, LOW);   // 역방향

    analogWrite(SPD1, duty1);

    // === 모터2 ===
    uint32_t cur2 = plsoCount2;
    uint32_t delta2 = cur2 - lastCount2;
    lastCount2 = cur2;

    float rps2 = (delta2 / 6.0) / 0.1;

    float error2 = abs(set_rps2) - rps2;
    integral2 += error2 * 0.1;
    float deriv2 = (error2 - lastError2) / 0.1;
    lastError2 = error2;

    float output2 = Kp*error2 + Ki*integral2 + Kd*deriv2;
    duty2 += output2;
    duty2 = constrain(duty2, 0, 255);

    if (set_rps2 >= 0) digitalWrite(RVS2, HIGH);
    else               digitalWrite(RVS2, LOW);

    analogWrite(SPD2, duty2);

    // === 디버그 출력 ===
    Serial.print("M1 RPS="); Serial.print(rps1);
    Serial.print(" Duty="); Serial.print(duty1);
    Serial.print(" | M2 RPS="); Serial.print(rps2);
    Serial.print(" Duty="); Serial.println(duty2);

    lastTime = now;
  }
}
