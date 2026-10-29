#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS 10
MCP_CAN CAN(CAN_CS);

const int TURN_THRESHOLD = 30;   // -15..15 = lurus (0), > 15 = kanan (2), < -15 = kiri (1)

char buf[12];
uint8_t len = 0;

void sendCommand(uint8_t cmd) {
  byte data[5];
  data[0] = cmd;

  unsigned long currentMicros = micros();
  memcpy(&data[1], &currentMicros, 4);

  CAN.sendMsgBuf(0x100, 0, 5, data);   // frame sama persis dengan kode lama Anda
}

void processLine() {
  buf[len] = 0;
  int e = atoi(buf);

  uint8_t cmd;
  if (e > TURN_THRESHOLD)       cmd = 2;   // kanan
  else if (e < -TURN_THRESHOLD) cmd = 1;   // kiri
  else                          cmd = 0;   // lurus

  sendCommand(cmd);

  Serial.print("Kirim: error ");
  Serial.print(e);
  Serial.print(" -> ");
  Serial.println(cmd);
}

void setup() {
  Serial.begin(115200);
  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init gagal, mencoba lagi...");
    delay(500);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("Master siap");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (len > 0) {          // baris selesai -> proses
        processLine();
        len = 0;
      }
    } else if ((c >= '0' && c <= '9') || c == '-') {
      if (len < sizeof(buf) - 1) buf[len++] = c;
    }
    // karakter lain (huruf, spasi, dll.) diabaikan
  }
}
