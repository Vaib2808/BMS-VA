#include <FlexCAN_T4.h>
#include "avr/pgmspace.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can2;
CAN_message_t msg;
char buf[40];
String checks;
int8_t dataTherm[10];
int value1, adcValue;
unsigned long value3;
uint32_t lastTransmissionTime = 0;
unsigned long highStartTime = 0;
bool isHigh = false; 
const int OUTPUT_PIN = 17;

void setup() {
  can1.begin();
  can1.setBaudRate(500000);
  can1.setMaxMB(16);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(canSniff);
  can1.mailboxStatus();
  can2.begin();
  can2.setBaudRate(500000);
}

void loop() {
  can1.events();
  static uint32_t timeout = millis();
  if (millis() - timeout > 2000) {
    sendtherm();
    timeout = millis();
  }
}

void canSniff(const CAN_message_t &msg)  {
  value3 = msg.id;
  if (msg.id == 0x1838F380 || msg.id == 0x1838F381) {
    Serial.print("CAN ID:  ");
    Serial.print(msg.id, HEX); 
    Serial.print(" ");
    for (int k = 0; k < 8; k++) {
      dataTherm[k] = (int8_t)msg.buf[k];
      Serial.print(dataTherm[k]);
      Serial.print(" ");
    }
    Serial.println();
    if (dataTherm[2] > 60) {
      if (!isHigh) {
        highStartTime = millis();
      } else {
        if (millis() - highStartTime > 1000) {
          digitalWrite(OUTPUT_PIN, HIGH);
        }
      }
  }
}
}

void sendtherm() {
  CAN_message_t msg;
  msg.id = 0x100;
  msg.len = 2;
  msg.buf[0] = dataTherm[4];
  msg.buf[1] = dataTherm[5];
  can2.write(msg);
}