#include <Arduino.h>
#include <math.h>

/*
 * 입력: "CMD M1:<val> M2:<val>\n"
 * - 여기서 <val>에 CMD_SCALE(기본 100)을 곱해 rps 목표로 사용
 * - 파이썬에서 이미 rps로 보낸다면 CMD_SCALE = 1.0f 로 설정하세요.
 */

// ===== 핀 설정 =====
const uint8_t SPD1  = 6;   // 모터1 PWM
const uint8_t RUN1  = 23;  // 모터1 Run enable (LOW=Run 가정)
const uint8_t RVS1  = 25;  // 모터1 방향
const uint8_t PLSO1 = 2;   // 모터1 PLSO (INT0)

const uint8_t SPD2  = 7;   // 모터2 PWM
const uint8_t RUN2  = 22;  // 모터2 Run enable (LOW=Run 가정)
const uint8_t RVS2  = 24;  // 모터2 방향
const uint8_t PLSO2 = 3;   // 모터2 PLSO (INT1)

// ===== RUN 논리 =====
const bool RUN1_ACTIVE_HIGH = false;
const bool RUN2_ACTIVE_HIGH = false;

// ===== 모터 피드백 카운터 =====
volatile uint32_t plsoCount1 = 0;
volatile uint32_t plsoCount2 = 0;
void isrPlso1() { plsoCount1++; }
void isrPlso2() { plsoCount2++; }

// ===== PID 파라미터/상태 =====
float Kp = 0.8f, Ki = 0.2f, Kd = 0.0f;
float set_rps1 = 0.0f;
float set_rps2 = 0.0f;

uint32_t lastCount1 = 0, lastCount2 = 0;
uint32_t lastPidTick = 0;

float integral1 = 0, integral2 = 0;
float lastError1 = 0, lastError2 = 0;
float duty1 = 0, duty2 = 0;

// ===== 펄스→속도 변환 =====
const float    PULSES_PER_REV  = 6.0f;   // 엔코더 1회전 펄스 수
const uint16_t PID_DT_MS       = 100;    // PID 주기(ms)

// ===== 제한/타임아웃/스케일 =====
const float    MAX_RPS         = 250.0f; // 목표 rps 상한(안전)
const uint16_t CMD_TIMEOUT_MS  = 3000;   // 명령 타임아웃(ms)
const float    CMD_SCALE       = 100.0f; // ★ 수신값에 곱할 스케일(예: 0.1~0.5 → 10~50)

// ===== 시리얼 상태 =====
String serialBuffer = "";
unsigned long lastCmdTime = 0;           // 마지막 유효 명령 수신 시각

// ===== 유틸 =====
inline void setRun(uint8_t pin, bool active, bool activeHigh) {
  digitalWrite(pin, activeHigh ? (active ? HIGH : LOW)
                               : (active ? LOW  : HIGH));
}

void applyMotorOutput(
  uint8_t runPin, uint8_t rvsPin, uint8_t spdPin,
  float set_rps, float duty, bool runActiveHigh
){
  if (set_rps == 0.0f) {
    setRun(runPin, false, runActiveHigh);
    analogWrite(spdPin, 0);
    digitalWrite(rvsPin, LOW);
  } else {
    setRun(runPin, true, runActiveHigh);
    digitalWrite(rvsPin, (set_rps >= 0) ? HIGH : LOW); // 방향
    analogWrite(spdPin, (int)duty);                    // 0~255
  }
}

// ===== 시리얼 명령 처리 =====
// 포맷: "CMD M1:<val> M2:<val>\n"
//  - <val>에 CMD_SCALE(100)을 곱해 rps 목표로 사용
void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (serialBuffer.startsWith("CMD")) {
        int m1_idx = serialBuffer.indexOf("M1:");
        int m2_idx = serialBuffer.indexOf("M2:");

        if (m1_idx >= 0 && m2_idx >= 0) {
          float v1 = serialBuffer.substring(m1_idx + 3, m2_idx).toFloat();
          float v2 = serialBuffer.substring(m2_idx + 3).toFloat();

          // ★ 스케일 적용
          v1 *= CMD_SCALE;
          v2 *= CMD_SCALE;

          // 안전 상한 적용
          v1 = constrain(v1, -MAX_RPS, MAX_RPS);
          v2 = constrain(v2, -MAX_RPS, MAX_RPS);

          set_rps1 = v1;
          set_rps2 = v2;

          lastCmdTime = millis(); // 유효 명령 수신 시각 갱신
        }
      }
      serialBuffer = "";
    } else {
      serialBuffer += c;
    }
  }
}

void setup() {
  pinMode(SPD1, OUTPUT); digitalWrite(SPD1, LOW);
  pinMode(SPD2, OUTPUT); digitalWrite(SPD2, LOW);
  pinMode(RUN1, OUTPUT);
  pinMode(RUN2, OUTPUT);
  pinMode(RVS1, OUTPUT); digitalWrite(RVS1, LOW);
  pinMode(RVS2, OUTPUT); digitalWrite(RVS2, LOW);
  pinMode(PLSO1, INPUT_PULLUP);
  pinMode(PLSO2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PLSO1), isrPlso1, RISING);
  attachInterrupt(digitalPinToInterrupt(PLSO2), isrPlso2, RISING);

  setRun(RUN1, false, RUN1_ACTIVE_HIGH);
  setRun(RUN2, false, RUN2_ACTIVE_HIGH);

  Serial.begin(115200);

  lastPidTick = millis();
  lastCmdTime = millis();
}

void loop() {
  handleSerial();

  // === 타임아웃: 일정 시간 수신 없으면 정지 ===
  if (millis() - lastCmdTime > CMD_TIMEOUT_MS) {
    set_rps1 = 0.0f;
    set_rps2 = 0.0f;
  }

  uint32_t now = millis();
  if (now - lastPidTick >= PID_DT_MS) {
    lastPidTick += PID_DT_MS;

    // --- 실제 rps 계산 ---
    uint32_t cur1 = plsoCount1;
    uint32_t delta1 = cur1 - lastCount1;
    lastCount1 = cur1;
    float rps1 = (delta1 / PULSES_PER_REV) / (PID_DT_MS / 1000.0f);

    uint32_t cur2 = plsoCount2;
    uint32_t delta2 = cur2 - lastCount2;
    lastCount2 = cur2;
    float rps2 = (delta2 / PULSES_PER_REV) / (PID_DT_MS / 1000.0f);

    // --- PID (절대속도 제어: 방향은 RVS 핀으로 처리) ---
    if (set_rps1 == 0.0f) {
      integral1 = 0.0f; lastError1 = 0.0f; duty1 = 0.0f;
    } else {
      float target1 = fabs(set_rps1);
      float error1  = target1 - rps1;
      integral1 += error1 * (PID_DT_MS / 1000.0f);
      float deriv1  = (error1 - lastError1) / (PID_DT_MS / 1000.0f);
      lastError1 = error1;

      float u1 = Kp * error1 + Ki * integral1 + Kd * deriv1;
      duty1 += u1;                                   // 적분형 누적
      duty1 = constrain(duty1, 0.0f, 255.0f);        // PWM 한계
    }

    if (set_rps2 == 0.0f) {
      integral2 = 0.0f; lastError2 = 0.0f; duty2 = 0.0f;
    } else {
      float target2 = fabs(set_rps2);
      float error2  = target2 - rps2;
      integral2 += error2 * (PID_DT_MS / 1000.0f);
      float deriv2  = (error2 - lastError2) / (PID_DT_MS / 1000.0f);
      lastError2 = error2;

      float u2 = Kp * error2 + Ki * integral2 + Kd * deriv2;
      duty2 += u2;
      duty2 = constrain(duty2, 0.0f, 255.0f);
    }

    // --- 출력 적용 ---
    applyMotorOutput(RUN1, RVS1, SPD1, set_rps1, duty1, RUN1_ACTIVE_HIGH);
    applyMotorOutput(RUN2, RVS2, SPD2, set_rps2, duty2, RUN2_ACTIVE_HIGH);
  }
}
