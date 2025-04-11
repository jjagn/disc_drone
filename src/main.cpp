#include "DShotRMT.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>

/// DSHOT
#define POT_PIN 23      // potentiometer pin
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

void mpuSetup(Adafruit_MPU6050 *mpu);

// DSHOT
const auto DSHOT_MODE = DSHOT300;

// Define the failsafe and initial throttle values
const auto FAILSAFE_THROTTLE = 999;
const auto INITIAL_THROTTLE = 48;

// Initialize a DShotRMT object for the motor
DShotRMT motor01(ESC_PIN);

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

void setup() {
  Serial.begin(115200);
  motor01.begin(DSHOT_MODE, ENABLE_BIDIRECTION, 14);
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
  // Serial.println("updating dshot to motor");
  motor01.send_dshot_value(INITIAL_THROTTLE);

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

    // send throttle to esc
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
