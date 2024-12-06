#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>

#define GYRO_Z_OFFSET 1.11 / 200

Adafruit_MPU6050 mpu;

float heading = 0;
float gyroZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
float gyroError = 0;

void setup(void) {
  Serial.begin(115200);

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop() {
  static int pacer = 0;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Serial.print("Z: ");
  // Serial.print(g.gyro.z);
  // Serial.println(" rad/s");

  // Serial.println("");
  gyroZ = g.gyro.z + GYRO_Z_OFFSET; // 16.4 LSB/deg/s
  // Serial.print("Gyro: ");
  // Serial.println(gyroZ);
  currentTime = millis();
  // Serial.print("Current time: ");
  // Serial.println(currentTime);
  elapsedTime = (currentTime - previousTime) / 1000; // /1000 to get seconds
  previousTime = currentTime;
  heading += gyroZ * elapsedTime;
  
    if (pacer++ > 200) {
    Serial.print("Heading: ");
    Serial.println(heading);
    pacer = 0;
  }
}
