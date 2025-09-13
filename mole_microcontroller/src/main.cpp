#include <Arduino.h>
#include "hardware_msgs.hpp"

#define LED_BUILTIN 2  // Pin 2

#define SERIAL_BAUDRATE 115200
#define SERIAL_TIMEOUT_MS 100

#define HEADER1 0xAA
#define HEADER2 0xBB
#define TYPE_VELOCITY 0x00
#define TYPE_ENCODER 0x01
#define TYPE_ENCODER_REQUEST 0x02
#define TYPE_PID 0x03

#define VELOCITY_PAYLOAD_SIZE 8
#define ENCODER_PAYLOAD_SIZE 8
#define ENCODER_REQUEST_PAYLOAD_SIZE 0
#define PID_PAYLOAD_SIZE 16

bool readPacket(Packet& pkt);
void handlePacket(const Packet& pkt);
void sendEncoderData(int32_t left_encoder, int32_t right_encoder);
void setPidSettings(int32_t p, int32_t i, int32_t d, int32_t o);
void setVelocity(int32_t left_velocity, int32_t right_velocity);

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  Serial.setTimeout(SERIAL_TIMEOUT_MS);
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 3; ++i) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

void loop() {
  Packet pkt;
  if (readPacket(pkt)) {
    handlePacket(pkt);
  }
}

bool readPacket(Packet& pkt) {
  static enum { WAIT_HEADER1, WAIT_HEADER2, WAIT_TYPE, WAIT_PAYLOAD, WAIT_CHECKSUM } state = WAIT_HEADER1;
  static size_t index = 0;
  static size_t payload_length = 0;

  while (Serial.available()) {
    uint8_t byte = Serial.read();
    switch (state) {
      case WAIT_HEADER1:
        if (byte == HEADER1) state = WAIT_HEADER2;
        break;
      case WAIT_HEADER2:
        if (byte == HEADER2) state = WAIT_TYPE;
        else state = WAIT_HEADER1;
        break;
      case WAIT_TYPE:
        pkt.type = byte;
        state = WAIT_PAYLOAD;
        index = 0;
        payload_length = 0;
        switch (pkt.type) {
          case TYPE_VELOCITY:
            payload_length = VELOCITY_PAYLOAD_SIZE;
            break;
          case TYPE_ENCODER:
            payload_length = ENCODER_PAYLOAD_SIZE;
            break;
          case TYPE_ENCODER_REQUEST:
            payload_length = ENCODER_REQUEST_PAYLOAD_SIZE;
            break;
          case TYPE_PID:
            payload_length = PID_PAYLOAD_SIZE;
            break;
        }
        break;
      case WAIT_PAYLOAD:
        if (index < payload_length) {
          pkt.payload[index++] = byte;
        } else {
          state = WAIT_CHECKSUM;
        }
        break;
      case WAIT_CHECKSUM:
        pkt.checksum = byte;
        // Validate checksum
        if (calculate_checksum((uint8_t *)&pkt, 2 + 1 + payload_length) == pkt.checksum) {
          state = WAIT_HEADER1;
          return true;
        } else {
          state = WAIT_HEADER1; // Invalid checksum, reset state
        }
        break;
    }
  }
  return false;
}

void handlePacket(const Packet& pkt) {
  switch (pkt.type) {
    case TYPE_VELOCITY: {
      if (sizeof(VelocityPacket) - 1 != 2 + 1 + VELOCITY_PAYLOAD_SIZE) return; // Size mismatch guard for accidental struct change
      VelocityPacket v_pkt;
      memcpy(&v_pkt, pkt.payload, VELOCITY_PAYLOAD_SIZE);
      setVelocity(v_pkt.left_velocity, v_pkt.right_velocity);
      break;
    }
    case TYPE_ENCODER_REQUEST: {
      // Currently, just send dummy encoder data since motor class not implemented
      sendEncoderData(12345, 67890);
      break;
    }
    case TYPE_PID: {
      if (sizeof(PIDPacket) - 1 != 2 + 1 + PID_PAYLOAD_SIZE) return; // Size mismatch guard for accidental struct change
      PIDPacket p_pkt;
      memcpy(&p_pkt, pkt.payload, PID_PAYLOAD_SIZE);
      setPidSettings(p_pkt.p, p_pkt.i, p_pkt.d, p_pkt.o);
      break;
    }
  }
}