#include "Arduino.h"
#include "esp32-hal.h"
#include "dshot.h"

void dshotSetup(rmt_obj_t **rmt_send) {
  // Serial.print("init starting\n");
  pinMode(POT_PIN, INPUT);
  pinMode(FWD_REV_PIN, INPUT_PULLUP);
  pinMode(ESC_PIN, OUTPUT);
  *rmt_send = rmtInit(ESC_PIN, TX_NO_RX, RMT_MEM_64);
  if (*rmt_send == NULL) {
    // Serial.println("init sender failed\n");
  }
  // Serial.print("init sender success\n");
  float realTick = rmtSetTick(*rmt_send, 12.5); // 12.5 nsec sample rate
  // Serial.printf("rmt_send tick set to: %fns\n", realTick);
  while (millis() < 3500) {
    dshotOutput(0, !TELEM_ON, *rmt_send);
    delay(1);
  }
  timer = timerBegin(TMR_CHAN, BOARD_MHZ,
                     COUNT_UP); // *18 returns pointer to timer structure
  timerAttachInterrupt(timer, &onTimer,
                       true); // *19 attatch timer interupt to onTimer routine
                              // defined above: edge type=true
  timerAlarmWrite(
      timer, 1000,
      true); // *20 1000 and prescale of 80 is a msec. reload the timeer=true
  timerAlarmEnable(timer);
}

void dshotOutput(uint16_t value, bool telemetry, rmt_obj_t *rmt_send) { // creates packet to send
  static rmt_data_t dshotPacket[16];    // structure defined in esp32_hal_h ?
  uint16_t packet;
  if (telemetry) {
    packet = (value << 1) | 1;
  } else {
    packet = (value << 1) | 0;
  }
  int csum = 0; // holds checksum part of packet?
  int csum_data = packet;
  for (int i = 0; i < 3; i++) {
    csum ^= csum_data;
    csum_data >>= 4;
  }
  csum &= 0xf;
  packet = (packet << 4) | csum;

  for (int i = 0; i < 16; i++) {
    dshotPacket[i].level0 = 1;
    dshotPacket[i].level1 = 0;
    if (packet & 0x8000) { // true for bit set
      dshotPacket[i].duration0 = LONG_TICKS;
      dshotPacket[i].duration1 = SHORT_TICKS;
    } else { // true for bit clear
      dshotPacket[i].duration0 = SHORT_TICKS;
      dshotPacket[i].duration1 = LONG_TICKS;
    }
    packet <<= 1;
  }
  rmtWrite(rmt_send, dshotPacket, 16);
  return;
}


void IRAM_ATTR onTimer() { // *7 ISR (interrupt service routine) for HW timer
  portENTER_CRITICAL_ISR(&timerMux); // *8
  interruptCounter++;                // *9
  dshotOutput(dshotUserInputValue, false,
              rmt_send_g); // send out the 16 bit digital command to ESC
  portEXIT_CRITICAL_ISR(&timerMux); // *10
} 

