#include <Arduino.h>

const int LED_PIN = 25; // On-board LED on Raspberry Pi Pico

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(300);
}
