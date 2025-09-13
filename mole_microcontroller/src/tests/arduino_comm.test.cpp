#include <Arduino.h>

#include "hardware_msgs.hpp"

#ifndef LED_BUILTIN
#define LED_BUILTIN 2  // Pin 2
#endif

void processSerialData();

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  processSerialData();
}

void processSerialData() {
  if (Serial.available()) {
    uint8_t header1 = Serial.read();
    if (header1 != 0xAA) return; // Invalid header
    if (Serial.available() < 1) return;
    uint8_t header2 = Serial.read();
    if (header2 != 0xBB) return; // Invalid header
    if (Serial.available() < 1) return;
    uint8_t type = Serial.read();
    if (type == 0x00) { // Velocity command
      while (Serial.available() < sizeof(int32_t) * 2 + 1); // Wait for full packet
      VelocityPacket packet;
      packet.header1 = header1;
      packet.header2 = header2;
      packet.type = type;
      Serial.readBytes((char *)&packet.left_velocity, sizeof(int32_t));
      Serial.readBytes((char *)&packet.right_velocity, sizeof(int32_t));
      packet.checksum = Serial.read();
      // Validate checksum
      uint8_t calculated_checksum = calculate_checksum((uint8_t *)&packet, sizeof(VelocityPacket) - 1);
      if (calculated_checksum != packet.checksum) return;
      // Process velocities (for demo, just blink LED)
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
    } else if (type == 0x03) { // PID settings
      while (Serial.available() < sizeof(int32_t) * 4 + 1); // Wait for full packet
      PIDPacket packet;
      packet.header1 = header1;
      packet.header2 = header2;
      packet.type = type;
      Serial.readBytes((char *)&packet.p, sizeof(int32_t));
      Serial.readBytes((char *)&packet.i, sizeof(int32_t));
      Serial.readBytes((char *)&packet.d, sizeof(int32_t));
      Serial.readBytes((char *)&packet.o, sizeof(int32_t));
      packet.checksum = Serial.read();
      // Validate checksum
      uint8_t calculated_checksum = calculate_checksum((uint8_t *)&packet, sizeof(PIDPacket) - 1);
      if (calculated_checksum != packet.checksum) return;
      // Process PID settings (for demo, just blink LED thrice)
      for (int i = 0; i < 3; ++i) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
      }
    } else if (type == 0x02) { // Request encoder data
      // Send back encoder data (for demo, send fixed values)
      EncoderPacket packet;
      packet.left_encoder = 20; // Example value
      packet.right_encoder = 40; // Example value
      packet.checksum = calculate_checksum((uint8_t *)&packet, sizeof(EncoderPacket) - 1);
      Serial.write((uint8_t *)&packet, sizeof(EncoderPacket));
      for (int i = 0; i < 2; ++i) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
      }
    } else {
      // Unknown packet type, ignore
      return;
    }
  }
}