#include <Arduino.h>
#include <unity.h>

// 1. Define your test functions here
void test_numeric_comparison(void) {
    int expected = 5;
    int actual = 5;
    TEST_ASSERT_EQUAL_INT(expected, actual);
}

void test_bool_condition(void) {
    bool condition = true;
    TEST_ASSERT_TRUE(condition);
}

// 2. Arduino setup function (runs once)
void setup() {
    // Wait for serial monitor to connect
    delay(2000); 
    
    // Initialize the Unity test framework
    UNITY_BEGIN();
    
    // Run your test cases
    RUN_TEST(test_numeric_comparison);
    RUN_TEST(test_bool_condition);
    
    // Signal the end of testing
    UNITY_END();
}

// 3. Arduino loop function (must be present but left empty)
void loop() {
    // Tests run once in setup(), nothing goes here
}