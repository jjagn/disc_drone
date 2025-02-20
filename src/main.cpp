#include "HardwareSerial.h"
#include "esp32-hal-adc.h"
#include <Arduino.h>
#include <esp32-hal.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

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

// PID
#define K_P 0
#define K_I 0
#define K_D 0.2 

// DShot 600 has frame rate of 600 kHz, which is 133 ticks of a 80MHz clock
#define SHORT_TICKS 44 // results in 555 nsec
#define LONG_TICKS 89 // results in 1111 nsec; total is 133 ticks of 80MHz clock

rmt_obj_t *rmt_send_g = NULL;

void mpuSetup(Adafruit_MPU6050 *mpu);
void updateDShot(float motorVal);
void dshotOutput(uint16_t value, bool telemetry, rmt_obj_t *rmt_send);
void dshotSetup(rmt_obj_t **rmt_send);

volatile uint16_t dshotUserInputValue = 0; // holds the pot reading for speed to be sent in packet
volatile int interruptCounter = 0; // *1 counts the interrupts from timer. volitile to make it persistent
int totalInterruptCounter; // *2
hw_timer_t *timer = NULL;  // *4
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; // *5 used to maintain sync between interupt
                                  // and loop code

void IRAM_ATTR onTimer() { // *7 ISR (interrupt service routine) for HW timer
  portENTER_CRITICAL_ISR(&timerMux); // *8
  interruptCounter++;                // *9
  dshotOutput(dshotUserInputValue, false,
              rmt_send_g); // send out the 16 bit digital command to ESC
  portEXIT_CRITICAL_ISR(&timerMux); // *10
} 

// nav etc.
#define GYRO_X_OFFSET -0.04
#define GYRO_Y_OFFSET 0.05
#define GYRO_Z_OFFSET 1.11 / 200

Adafruit_MPU6050 mpu;

float pitch = 0;
float roll = 0;
float heading = 0;
float gyroX;
float gyroY;
float gyroZ;
float delta_t, currentTime, previousTime;
float gyroError = 0;
int pot = 0;

void setup() {          // *14
  Serial.begin(115200); // *16
  dshotSetup(&rmt_send_g);
  mpuSetup(&mpu);
}

void loop() {
  static int pacer = 0;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  gyroX = g.gyro.x + GYRO_X_OFFSET; // 16.4 LSB/deg/s
  gyroY = g.gyro.y + GYRO_Y_OFFSET; // 16.4 LSB/deg/s
  gyroZ = g.gyro.z + GYRO_Z_OFFSET; // 16.4 LSB/deg/s
  currentTime = millis();
  delta_t = (currentTime - previousTime) / 1000; // /1000 to get seconds
  previousTime = currentTime;
  roll += gyroY * delta_t;
  pitch += gyroX * delta_t;
  heading += gyroZ * delta_t;
  
  // PID
  static float setpoint = 0;
  static float integrator = 0;
  static float prevError = 0;
  float output = 0;
  float error = setpoint - roll;

  // P
  output = error * K_P;

  // I 
  integrator += error * delta_t;
  output += integrator * K_I;

  // D
  float derivative = (error - prevError) / delta_t;
  prevError = error;
  output += derivative * K_D;

  // updateDShot(output);
  // updateDShot(1);


  if (pacer++ > 200) {
    pacer = 0;
    Serial.print("Heading: ");
    Serial.print(heading * RAD_TO_DEG);
    Serial.print(",");
    Serial.print("Pitch: ");
    Serial.print(pitch * RAD_TO_DEG);
    Serial.print(",");
    Serial.print("Roll: ");
    Serial.print(roll * RAD_TO_DEG);
    Serial.print(",");
    Serial.print("Gyro X: ");
    Serial.print(gyroX);
    Serial.print(",");
    Serial.print("Gyro Y: ");
    Serial.print(gyroY);
    Serial.print(",");

    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(",");
    Serial.print("Integrator: ");
    Serial.print(integrator);
    Serial.print(",");
    Serial.print("Derivative: ");
    Serial.print(derivative);
    Serial.print(",");
    Serial.print("Output: ");
    Serial.print(output);
    Serial.println("");
  }
}

void mpuSetup(Adafruit_MPU6050 *mpu) {
  if (!mpu->begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu->setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu->setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu->setFilterBandwidth(MPU6050_BAND_260_HZ);
  Serial.println("");
  delay(100);
}

void updateDShot(float motorVal) {
      // uint16_t dUR = analogRead(POT_PIN); // read the pot pin 0 - 4095
      dshotUserInputValue = map(motorVal*1000, 0, 1540, 48, 1047); // 1 to 47 commands are reserved
      // dshotUserInputValue = map(dUR, 0, 4096, 48, 1047); // 1 to 47 commands are reserved
      if (dshotUserInputValue < 50 || dshotUserInputValue > 60000)
        dshotUserInputValue = 0;
      if (interruptCounter > 0) { // *27 true if it's time to enter interrupt code
        portENTER_CRITICAL(&timerMux); // *29
        interruptCounter--;            // *30
        portEXIT_CRITICAL(&timerMux);  // *31
        totalInterruptCounter++;       // *33
        // Serial.printf("pot: %i   totalinterruptcounter: %i \n", dshotUserInputValue, totalInterruptCounter); // *35,36
      }
}

void dshotSetup(rmt_obj_t **rmt_send, int outputPin) {
  Serial.print("init starting\n");
  pinMode(POT_PIN, INPUT);
  pinMode(FWD_REV_PIN, INPUT_PULLUP);
  pinMode(outputPin, OUTPUT);
  *rmt_send = rmtInit(ESC_PIN, TX_NO_RX, RMT_MEM_64);
  if (*rmt_send == NULL) {
    Serial.println("init sender failed\n");
  }
  Serial.print("init sender success\n");
  float realTick = rmtSetTick(*rmt_send, 12.5); // 12.5 nsec sample rate
  Serial.printf("rmt_send tick set to: %fns\n", realTick);
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

void dshotOutput(uint16_t value, bool telemetry,
                 rmt_obj_t *rmt_send) { // creates packet to send
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
