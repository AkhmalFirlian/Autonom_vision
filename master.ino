#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS 10
MCP_CAN CAN(CAN_CS);

const int TURN_THRESHOLD = 15;   // px, sama dengan di Python

char rxBuf[16];
byte rxLen = 0;

void kirimKeCAN(int error) {

  byte cmd = 0;                                  // lurus
  if (error >  TURN_THRESHOLD) cmd = 2;          // kanan
  if (error < -TURN_THRESHOLD) cmd = 1;          // kiri

  byte data[3];
  data[0] = cmd;
  data[1] = (byte)(error & 0xFF);
  data[2] = (byte)((error >> 8) & 0xFF);

  CAN.sendMsgBuf(0x100, 0, 3, data);

  Serial.print("Kirim: error ");
  Serial.print(error);
  Serial.print(" cmd ");
  Serial.println(cmd);
}

void setup() {
  Serial.begin(115200);
  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) { delay(500); }
  CAN.setMode(MCP_NORMAL);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      if (rxLen > 0) {
        rxBuf[rxLen] = 0;
        kirimKeCAN(atoi(rxBuf));
      }
      rxLen = 0;
    }
    else if (c != '\r') {
      if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = c;
      else rxLen = 0;
    }
  }
}
