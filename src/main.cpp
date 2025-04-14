#include "esp32-hal-adc.h"
#include <Arduino.h>

// TEST
#include <hal/gpio_hal.h>

#include <DShotRMT.h>

#define THROTTLE_MIN 47
#define THROTTLE_MAX 2047

#define ESC_ARM_TIME 1000

// Define the GPIO pin connected to the motor and the DShot protocol used
const auto MOTOR01_PIN = 16;
const auto MOTOR02_PIN = 17;
const auto DSHOT_MODE = DSHOT150;

// Define the failsafe and initial throttle values
const auto FAILSAFE_THROTTLE = 0;
const auto INITIAL_THROTTLE = 48;

#define POT_PIN 4

DShotRMT ESC_01(MOTOR01_PIN); // MOTOR01_PIN
DShotRMT ESC_02(MOTOR02_PIN); // MOTOR02_PIN

void setup() {
	Serial.begin(115200);
	ESC_01.begin(DSHOT_MODE, NO_BIDIRECTION, 14);
	ESC_02.begin(DSHOT_MODE, NO_BIDIRECTION, 14);
	pinMode(POT_PIN, INPUT);
	Serial.println("starting");
    while (analogRead(POT_PIN) >= 100) {
        Serial.println("Turn down throttle!");
    }
    // arm
    unsigned long time_now = millis();
    while (millis() - time_now >= ESC_ARM_TIME) {
        ESC_01.send_dshot_value(0);
        ESC_02.send_dshot_value(0);
    }
}

void loop() {
	uint16_t rpm_set_now = 400;
	uint16_t throttle_command = 400;
	{
		uint16_t tpot_value = analogRead(POT_PIN);
		uint16_t pot_value = abs(tpot_value - pot_value) > 50 ? tpot_value : pot_value;

		throttle_command = map(pot_value, 0, 4095, THROTTLE_MIN, THROTTLE_MAX);
		if (throttle_command > THROTTLE_MAX)
			throttle_command = THROTTLE_MAX;
		if (throttle_command < THROTTLE_MIN)
			throttle_command = THROTTLE_MIN;
		Serial.println(throttle_command);
        ESC_01.send_dshot_value(throttle_command);
        ESC_02.send_dshot_value(throttle_command);

	}
}
