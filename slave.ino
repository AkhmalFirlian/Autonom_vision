// =====================================================
// MOBIL RC + AUTO (Arduino Mega 2560) -- v7
// CH4 stick kiri-kanan (menempel): KIRI = AUTO, KANAN = MANUAL
// MANUAL : CH3 gas, CH2 arah (atas maju, bawah mundur), CH1 steering
// AUTO   : steering dari CAN, gas dari CH3 (atau tetap)
// =====================================================
#include <SPI.h>
#include <mcp_can.h>

// ---------- Motor gerak kiri ----------
const int R_PWM = 6;
const int L_PWM = 7;
const int ENA   = 5;

// ---------- Motor gerak kanan ----------
const int R_PWM2 = 9;
const int L_PWM2 = 10;
const int ENA2   = 8;

// ---------- Motor steering ----------
const int R_PWM_STEER = 12;
const int L_PWM_STEER = 13;
const int ENA_STEER   = 11;
const int POT_PIN     = A0;

// ---------- CAN ----------
const int CAN_CS = 25;
MCP_CAN CAN(CAN_CS);

// ---------- Kanal remote (0 = CH1 ... 5 = CH6) ----------
const int CH_GAS   = 2;   // CH3
const int CH_ARAH  = 1;   // CH2
const int CH_STEER = 0;   // CH1
const int CH_MODE  = 3;   // CH4

// ---------- Kalibrasi remote ----------
const int  GAS_MIN        = 1115;
const int  GAS_MAX        = 2000;
const int  GAS_DEADBAND   = 60;

const int  ARAH_BAWAH     = 1200;
const int  ARAH_ATAS      = 1800;
const bool BALIK_ARAH     = false;

const int  MODE_KIRI      = 1200;   // di bawah ini = AUTO
const int  MODE_KANAN     = 1800;   // di atas ini  = MANUAL
const bool BALIK_MODE     = false;

const int  RC_STEER_MIN     = 1000;
const int  RC_STEER_TENGAH  = 1618;
const int  RC_STEER_MAX     = 2000;
const int  RC_STEER_DEAD    = 30;
const bool BALIK_STEER      = false;

const int  RAMP_STEP      = 5;

// ---------- Kalibrasi steering (potensiometer) ----------
const int   STEER_POT_KIRI    = 588;
const int   STEER_POT_TENGAH  = 650;
const int   STEER_POT_KANAN   = 726;
const int   STEER_LIMIT_KIRI  = 563;    // sesuaikan dengan batas mekanik
const int   STEER_LIMIT_KANAN = 751;    // sesuaikan dengan batas mekanik
const int   POT_TOLERANSI     = 5;

float Kp = 3.7;
float Ki = 0.05;
float Kd = 0.35;
const float INTEGRAL_MAX = 1500;

// ---------- Pengaturan AUTO ----------
const bool  AUTO_PAKAI_ERROR  = true;   // true = belok halus (jika pesan CAN membawa error)
                                        // false = 3 keputusan lurus/kiri/kanan
const float AUTO_GAIN         = 0.3;    // satuan pot per piksel
const int   AUTO_DEADBAND_PX  = 15;
const unsigned long AUTO_TIMEOUT_MS = 500;

const bool  AUTO_GAS_TETAP    = false;  // false = gas dari CH3, true = kecepatan tetap
const int   AUTO_SPEED_PWM    = 80;

const bool  DEBUG_PRINT = true;

// ---------- Data remote (volatile karena diubah di ISR) ----------
volatile unsigned long timer[7];
volatile byte          last_channel[6];
volatile int           input[6];
volatile unsigned long lastPulse = 0;

int  currentSpeed = 0;
bool armed = false;
bool mintaMundur = false;
bool mundurMode  = false;
bool mintaAuto   = false;
bool autoMode    = false;

// ---------- CAN ----------
bool canOK = false;
bool canPernah = false;
int  canCmd = 0;
int  canError = 0;
bool canAdaError = false;
unsigned long canMs = 0;

// ---------- PID steering ----------
float error = 0, lastError = 0, integral = 0, derivative = 0;
int dbgPot = 0, dbgTarget = STEER_POT_TENGAH;

// =====================================================
// INTERRUPT REMOTE
// =====================================================
ISR(PCINT2_vect) {

  timer[0] = micros();
  lastPulse = timer[0];

  // CH1
  if ((last_channel[0] == 0) && (PINK & B00100000)) {
    last_channel[0] = 1;  timer[1] = timer[0];
  } else if ((last_channel[0] == 1) && !(PINK & B00100000)) {
    last_channel[0] = 0;  input[0] = timer[0] - timer[1];
  }

  // CH2
  timer[0] = micros();
  if ((last_channel[1] == 0) && (PINK & B00010000)) {
    last_channel[1] = 1;  timer[2] = timer[0];
  } else if ((last_channel[1] == 1) && !(PINK & B00010000)) {
    last_channel[1] = 0;  input[1] = timer[0] - timer[2];
  }

  // CH3
  timer[0] = micros();
  if ((last_channel[2] == 0) && (PINK & B00001000)) {
    last_channel[2] = 1;  timer[3] = timer[0];
  } else if ((last_channel[2] == 1) && !(PINK & B00001000)) {
    last_channel[2] = 0;  input[2] = timer[0] - timer[3];
  }

  // CH4
  timer[0] = micros();
  if ((last_channel[3] == 0) && (PINK & B00000100)) {
    last_channel[3] = 1;  timer[4] = timer[0];
  } else if ((last_channel[3] == 1) && !(PINK & B00000100)) {
    last_channel[3] = 0;  input[3] = timer[0] - timer[4];
  }

  // CH5
  timer[0] = micros();
  if ((last_channel[4] == 0) && (PINK & B00000010)) {
    last_channel[4] = 1;  timer[5] = timer[0];
  } else if ((last_channel[4] == 1) && !(PINK & B00000010)) {
    last_channel[4] = 0;  input[4] = timer[0] - timer[5];
  }

  // CH6
  timer[0] = micros();
  if ((last_channel[5] == 0) && (PINK & B00000001)) {
    last_channel[5] = 1;  timer[6] = timer[0];
  } else if ((last_channel[5] == 1) && !(PINK & B00000001)) {
    last_channel[5] = 0;  input[5] = timer[0] - timer[6];
  }
}

// =====================================================
// CAN: terima pesan dari master
// =====================================================
void bacaCAN() {
  if (!canOK) return;

  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    unsigned long canId;
    byte len;
    byte buf[8];
    CAN.readMsgBuf(&canId, &len, buf);

    if (len >= 1) {
      canCmd = buf[0];                       // 0 = lurus, 1 = kiri, 2 = kanan
      if (len == 3) {                        // master baru: cmd + error (int16)
        canError = (int16_t)((int)buf[1] | ((int)buf[2] << 8));
        canAdaError = true;
      } else {                               // master lama: hanya cmd
        canError = 0;
        canAdaError = false;
      }
      canMs = millis();
      canPernah = true;
    }
  }
}

// =====================================================
// SERIAL MONITOR (tiap 100 ms)
// =====================================================
void menampilkan_status() {
  if (!DEBUG_PRINT) return;

  static unsigned long tPrint = 0;
  if (millis() - tPrint < 100) return;
  tPrint = millis();

  for (int ch = 0; ch < 4; ch++) {
    noInterrupts();
    int v = input[ch];
    interrupts();
    Serial.print(v);
    Serial.print("\t");
  }
  Serial.print(autoMode ? "AUTO" : "MANUAL");
  Serial.print("\t");
  Serial.print(mundurMode ? "MUNDUR" : "MAJU");
  Serial.print("\t");
  Serial.print(currentSpeed);
  Serial.print("\t");
  Serial.print(dbgPot);
  Serial.print("\t");
  Serial.println(dbgTarget);
}

// =====================================================
// FUNGSI BANTU
// =====================================================
void stopSteering() {
  digitalWrite(R_PWM_STEER, LOW);
  digitalWrite(L_PWM_STEER, LOW);
  analogWrite(ENA_STEER, 0);
}

void stopSemua() {
  analogWrite(ENA, 0);
  analogWrite(ENA2, 0);
  stopSteering();
  currentSpeed = 0;
  armed = false;
  integral = 0;
}

void setMotorGerak(int speed) {   // -255..255
  if (speed > 0) {                // MAJU
    digitalWrite(R_PWM, HIGH);  digitalWrite(L_PWM, LOW);
    digitalWrite(R_PWM2, LOW);  digitalWrite(L_PWM2, HIGH);
    analogWrite(ENA, speed);
    analogWrite(ENA2, speed);
  } else if (speed < 0) {         // MUNDUR
    digitalWrite(R_PWM, LOW);   digitalWrite(L_PWM, HIGH);
    digitalWrite(R_PWM2, HIGH); digitalWrite(L_PWM2, LOW);
    analogWrite(ENA, -speed);
    analogWrite(ENA2, -speed);
  } else {
    analogWrite(ENA, 0);
    analogWrite(ENA2, 0);
  }
}

// PID posisi steering (dari kode slave, ditambah batas aman)
void pidSteering(int target) {

  int posisi = analogRead(POT_PIN);
  dbgPot = posisi;
  dbgTarget = target;

  error = target - posisi;

  if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) integral = 0;
  integral += error;
  if (integral >  INTEGRAL_MAX) integral =  INTEGRAL_MAX;
  if (integral < -INTEGRAL_MAX) integral = -INTEGRAL_MAX;

  derivative = error - lastError;
  float PID = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  int outputPWM = abs(PID);
  if (outputPWM > 255) outputPWM = 255;
  if (abs(error) > 50)      { if (outputPWM < 100) outputPWM = 100; }
  else if (abs(error) > 20) { if (outputPWM < 70)  outputPWM = 70; }
  else                      { if (outputPWM < 40)  outputPWM = 40; }

  if (abs(error) <= POT_TOLERANSI) {
    stopSteering();
    integral = 0;
  }
  else if (error > 0) {                              // ke KANAN
    if (posisi >= STEER_LIMIT_KANAN) {
      stopSteering();
    } else {
      digitalWrite(R_PWM_STEER, LOW);
      digitalWrite(L_PWM_STEER, HIGH);
      analogWrite(ENA_STEER, outputPWM);
    }
  }
  else {                                             // ke KIRI
    if (posisi <= STEER_LIMIT_KIRI) {
      stopSteering();
    } else {
      digitalWrite(R_PWM_STEER, HIGH);
      digitalWrite(L_PWM_STEER, LOW);
      analogWrite(ENA_STEER, outputPWM);
    }
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {

  Serial.begin(115200);

  pinMode(R_PWM, OUTPUT);        pinMode(L_PWM, OUTPUT);        pinMode(ENA, OUTPUT);
  pinMode(R_PWM2, OUTPUT);       pinMode(L_PWM2, OUTPUT);       pinMode(ENA2, OUTPUT);
  pinMode(R_PWM_STEER, OUTPUT);  pinMode(L_PWM_STEER, OUTPUT);  pinMode(ENA_STEER, OUTPUT);

  digitalWrite(R_PWM, LOW);        digitalWrite(L_PWM, LOW);
  digitalWrite(R_PWM2, LOW);       digitalWrite(L_PWM2, LOW);
  stopSemua();

  // CAN: coba beberapa kali, jika gagal mode manual tetap bisa dipakai
  for (int i = 0; i < 5 && !canOK; i++) {
    if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) canOK = true;
    else delay(300);
  }
  if (canOK) {
    CAN.setMode(MCP_NORMAL);
    if (DEBUG_PRINT) Serial.println("CAN OK");
  } else {
    if (DEBUG_PRINT) Serial.println("CAN GAGAL - hanya MANUAL yang berfungsi");
  }

  // Interrupt remote (A8 - A13)
  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT16);  // A8  -> CH6
  PCMSK2 |= (1 << PCINT17);  // A9  -> CH5
  PCMSK2 |= (1 << PCINT18);  // A10 -> CH4
  PCMSK2 |= (1 << PCINT19);  // A11 -> CH3
  PCMSK2 |= (1 << PCINT20);  // A12 -> CH2
  PCMSK2 |= (1 << PCINT21);  // A13 -> CH1

  if (DEBUG_PRINT) Serial.println("CH1\tCH2\tCH3\tCH4\tMODE\tARAH\tPWM\tPOT\tTARGET");
}

// =====================================================
// LOOP
// =====================================================
void loop() {

  bacaCAN();
  menampilkan_status();
  delay(10);

  // ---------- Salin data remote dengan aman ----------
  noInterrupts();
  int gas    = input[CH_GAS];
  int arah   = input[CH_ARAH];
  int steer  = input[CH_STEER];
  int modeIn = input[CH_MODE];
  unsigned long t = lastPulse;
  interrupts();

  // ---------- FAILSAFE REMOTE (semua mode) ----------
  bool sinyalOK = (gas    >= 900 && gas    <= 2100) &&
                  (arah   >= 900 && arah   <= 2100) &&
                  (steer  >= 900 && steer  <= 2100) &&
                  (modeIn >= 900 && modeIn <= 2100) &&
                  ((micros() - t) < 100000UL);

  if (!sinyalOK) {
    stopSemua();
    mintaMundur = false;
    mundurMode  = false;
    mintaAuto   = false;
    autoMode    = false;
    return;
  }

  // ---------- GAS MANUAL (CH3) ----------
  int kecepatan = 0;
  if (gas > GAS_MIN + GAS_DEADBAND) {
    kecepatan = map(gas, GAS_MIN + GAS_DEADBAND, GAS_MAX, 0, 255);
    kecepatan = constrain(kecepatan, 0, 255);
  }

  if (kecepatan == 0) armed = true;
  if (!armed) kecepatan = 0;

  // ---------- PEMILIH MODE (CH4, menempel) ----------
  bool stickKiri  = (modeIn < MODE_KIRI);
  bool stickKanan = (modeIn > MODE_KANAN);
  if (BALIK_MODE) {
    bool tmp = stickKiri;
    stickKiri  = stickKanan;
    stickKanan = tmp;
  }

  if (stickKiri)  mintaAuto = true;    // kiri  -> AUTO
  if (stickKanan) mintaAuto = false;   // kanan -> MANUAL

  if (!mintaAuto) {
    if (autoMode) {                    // ambil alih langsung
      autoMode = false;
      armed = false;
    }
  } else {
    if (!autoMode && kecepatan == 0) autoMode = true;
  }

  // ---------- ARAH (CH2, menempel) + TARGET KECEPATAN ----------
  bool stickBawah = (arah < ARAH_BAWAH);
  bool stickAtas  = (arah > ARAH_ATAS);
  if (BALIK_ARAH) {
    bool tmp = stickBawah;
    stickBawah = stickAtas;
    stickAtas  = tmp;
  }

  if (stickBawah) mintaMundur = true;
  if (stickAtas)  mintaMundur = false;

  if (kecepatan == 0) mundurMode = mintaMundur;

  int target = mundurMode ? -kecepatan : kecepatan;
  int steerTarget = STEER_POT_TENGAH;

  if (autoMode) {
    // ================= MODE AUTO =================
    bool segar = canPernah && ((millis() - canMs) < AUTO_TIMEOUT_MS);

    if (AUTO_GAS_TETAP) {
      target = segar ? AUTO_SPEED_PWM : 0;
      mundurMode = false;
    }

    if (segar) {
      if (AUTO_PAKAI_ERROR && canAdaError) {
        int e = canError;
        if (abs(e) < AUTO_DEADBAND_PX) e = 0;
        steerTarget = STEER_POT_TENGAH + (int)(e * AUTO_GAIN);
      } else {
        if (canCmd == 1)      steerTarget = STEER_POT_KIRI;
        else if (canCmd == 2) steerTarget = STEER_POT_KANAN;
        else                  steerTarget = STEER_POT_TENGAH;
      }
    }
  }
  else {
    // ================= MODE MANUAL =================
    int smin = RC_STEER_MIN;
    int smax = RC_STEER_MAX;
    if (BALIK_STEER) {
      steer = 2 * RC_STEER_TENGAH - steer;
      smin  = 2 * RC_STEER_TENGAH - RC_STEER_MAX;
      smax  = 2 * RC_STEER_TENGAH - RC_STEER_MIN;
    }

    if (steer > RC_STEER_TENGAH + RC_STEER_DEAD) {
      steerTarget = map(steer, RC_STEER_TENGAH + RC_STEER_DEAD, smax,
                        STEER_POT_TENGAH, STEER_POT_KANAN);
    }
    else if (steer < RC_STEER_TENGAH - RC_STEER_DEAD) {
      steerTarget = map(steer, RC_STEER_TENGAH - RC_STEER_DEAD, smin,
                        STEER_POT_TENGAH, STEER_POT_KIRI);
    }
  }

  steerTarget = constrain(steerTarget, STEER_POT_KIRI, STEER_POT_KANAN);

  // ---------- Ramping gerak ----------
  if (currentSpeed < target)      currentSpeed = min(currentSpeed + RAMP_STEP, target);
  else if (currentSpeed > target) currentSpeed = max(currentSpeed - RAMP_STEP, target);

  setMotorGerak(currentSpeed);
  pidSteering(steerTarget);
}
