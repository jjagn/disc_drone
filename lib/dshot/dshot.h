#ifndef DSHOT_H
#define DSHOT_H
#include "Arduino.h"
#include "esp32-hal.h"

/// DSHOT
#define POT_PIN 4      // potentiometer pin
#define ESC_PIN 16     // ESC control pin
#define FWD_REV_PIN 17 // HIGH is forward, LOW is reverse
#define MOTOR_POLES 14 // seems to be correct for my 2200KV motor
#define TMR_CHAN 0     // HW timer channel
#define BOARD_MHZ 80   // used as prescale in timerBegin call
#define COUNT_UP true  // HW timer count direction
#define TX_NO_RX true  // only send data to ESC for now
#define TELEM_ON true  // but we'll be commanding off for now

// DShot 600 has frame rate of 600 kHz, which is 133 ticks of a 80MHz clock
#define SHORT_TICKS 44 // results in 555 nsec
#define LONG_TICKS 89 // results in 1111 nsec; total is 133 ticks of 80MHz clock


hw_timer_t *timer = NULL;
void dshotSetup(rmt_obj_t **rmt_send);
void dshotOutput(uint16_t value, bool telemetry, rmt_obj_t *rmt_send);
void IRAM_ATTR onTimer();

volatile uint16_t dshotUserInputValue = 0; // holds the pot reading for speed to be sent in packet
volatile int interruptCounter = 0; // *1 counts the interrupts from timer. volitile to make it persistent
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; // *5 used to maintain sync between interupt  and loop code
rmt_obj_t *rmt_send_g = NULL;

#endif
