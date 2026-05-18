#include <Arduino.h>
#include <unity.h>

// Teensy 4.1 built-in LED
#define LED_PIN 13

// ─────────────────────────────────────────────
//  Test Cases
// ─────────────────────────────────────────────

// Verify LED pin can be driven HIGH and read back HIGH
void test_led_on(void) {
    digitalWrite(LED_PIN, HIGH);
    delay(10); // settle
    TEST_ASSERT_EQUAL(HIGH, digitalRead(LED_PIN));
}

// Verify LED pin can be driven LOW and read back LOW
void test_led_off(void) {
    digitalWrite(LED_PIN, LOW);
    delay(10); // settle
    TEST_ASSERT_EQUAL(LOW, digitalRead(LED_PIN));
}

// Visual blink — not a pass/fail assertion, just confirms hardware is alive
void test_led_blink(void) {
    for (int i = 0; i < 5; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_PIN, LOW);
        delay(200);
    }
    TEST_PASS(); // always passes — visual confirmation only
}

// ─────────────────────────────────────────────
//  Unity entry point
// ─────────────────────────────────────────────

void setup() {
    delay(2000); // wait for serial monitor

    pinMode(LED_PIN, OUTPUT);

    UNITY_BEGIN();
    RUN_TEST(test_led_on);
    RUN_TEST(test_led_off);
    RUN_TEST(test_led_blink);
    UNITY_END();
}

void loop() {}