#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS 25
#define PWM_3 11
#define RPWM_3 12
#define LPWM_3 13
#define POT A0

MCP_CAN CAN(CAN_CS);

int target = 650;
const int tolerance = 5;
float Kp = 3.7; float Ki = 0.05; float Kd = 0.35;
float error = 0; float lastError = 0; float integral = 0;
float derivative = 0;
const float integralMax = 1500;
byte lastCommand = 255;

void setup() {
  Serial.begin(115200);
  pinMode(PWM_3, OUTPUT); pinMode(RPWM_3, OUTPUT); pinMode(LPWM_3, OUTPUT);

  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN BUS Init Failed");
    delay(500);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("READY");
}

void loop() {
  // ===== RECEIVE CAN =====
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    unsigned long canId;
    byte len;
    byte buf[8];
    CAN.readMsgBuf(&canId, &len, buf);

    // Bagian Delay di sini sudah dihapus
    
    if (buf[0] != lastCommand) {
      lastCommand = buf[0];
      switch (buf[0]) {
        case 0: target = 650; Serial.println("Lurus"); break;
        case 1: target = 588; Serial.println("Kiri"); break;
        case 2: target = 726; Serial.println("Kanan"); break;
      }
      Serial.print("Target: "); Serial.println(target);
    }
  }

  // ===== PID =====
  int posisi = analogRead(POT);
  error = target - posisi;
  if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) integral = 0;
  integral += error;
  if (integral > integralMax) integral = integralMax;
  if (integral < -integralMax) integral = -integralMax;
  derivative = error - lastError;
  float PID = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  int outputPWM = abs(PID);
  if (outputPWM > 255) outputPWM = 255;
  if (abs(error) > 50) { if (outputPWM < 100) outputPWM = 100; }
  else if (abs(error) > 20) { if (outputPWM < 70) outputPWM = 70; }
  else { if (outputPWM < 40) outputPWM = 40; }

  // ===== MOTOR =====
  if (abs(error) <= tolerance) {
    digitalWrite(RPWM_3, LOW); digitalWrite(LPWM_3, LOW); analogWrite(PWM_3, 0);
    integral = 0;
  } else if (error > 0) {
    digitalWrite(RPWM_3, LOW); digitalWrite(LPWM_3, HIGH); analogWrite(PWM_3, outputPWM);
  } else {
    digitalWrite(RPWM_3, HIGH); digitalWrite(LPWM_3, LOW); analogWrite(PWM_3, outputPWM);
  }

  // ===== MONITORING =====
  static unsigned long timer = 0;
  if (millis() - timer > 50) {
    Serial.print("Waktu:"); Serial.print(millis());
    Serial.print(" | Posisi:"); Serial.print(posisi);
    Serial.print(" | Target:"); Serial.print(target);
    Serial.print(" | Error:"); Serial.println(error);
    timer = millis();
  }
  delay(10);
}
