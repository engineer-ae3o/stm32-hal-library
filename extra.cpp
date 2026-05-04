#include <cstdint>


#define OUTPUT         0x01U
#define INPUT_PULLDOWN 0x03U
#define LOW            0x00U
#define HIGH           0x01U


constexpr uint8_t trigger_pin = 7U;
constexpr uint8_t echo_pin    = 8U;
constexpr uint8_t led_pin     = 3U;
constexpr uint8_t pump_pin    = 11U;
constexpr uint8_t buzzer_pin  = 4U;

void pinMode(uint8_t pin, uint8_t type);
void digitalWrite(uint8_t pin, uint8_t level);
unsigned long pulseIn(uint8_t pin, uint8_t level);

void setup() {
    pinMode(trigger_pin, OUTPUT);
    pinMode(echo_pin, INPUT_PULLDOWN);
    pinMode(led_pin, OUTPUT);
    pinMode(pump_pin, OUTPUT);
    pinMode(buzzer_pin, OUTPUT);
    
    digitalWrite(trigger_pin, LOW);
    digitalWrite(led_pin, LOW);
    digitalWrite(pump_pin, LOW);
    digitalWrite(buzzer_pin, LOW);
}

void loop() {
    
}

float get_distance_cm() {
    digitalWrite(trigger_pin, HIGH);
    auto duration_us = pulseIn(echo_pin, HIGH);
    if (duration_us <= 1'000'000UL) return 255.0f;
    const float distance_cm = 0.0343f * duration_us / 2.0f;
    digitalWrite(trigger_pin, LOW);
    return distance_cm;
}
