#pragma once

#include <cstdint>
#include <cstddef>

#define MAX_PAYLOAD_SIZE 16

#pragma pack(push, 1)

struct Packet {
  uint8_t header1 = 0xAA;
  uint8_t header2 = 0xBB;
  uint8_t type; // 0x00 for velocity command, 0x01 for encoder data, 0x03 for PID settings
  uint8_t payload[MAX_PAYLOAD_SIZE];
  uint8_t checksum;
};
struct VelocityPacket {
  uint8_t header1 = 0xAA;
  uint8_t header2 = 0xBB;
  uint8_t type = 0x00; // 0x00 for velocity command
  int32_t left_velocity;
  int32_t right_velocity;
  uint8_t checksum;
};

struct EncoderPacket {
  uint8_t header1 = 0xAA;
  uint8_t header2 = 0xBB;
  uint8_t type = 0x01; // 0x01 for encoder data
  int32_t left_encoder;
  int32_t right_encoder;
  uint8_t checksum;
};
struct PIDPacket {
  uint8_t header1 = 0xAA;
  uint8_t header2 = 0xBB;
  uint8_t type = 0x03; // 0x03 for PID settings
  int32_t p;
  int32_t i;
  int32_t d;
  int32_t o;
  uint8_t checksum;
};
#pragma pack(pop)

uint8_t calculate_checksum(const uint8_t* data, size_t length) {
  uint8_t sum = 0;
  for (size_t i = 0; i < length; ++i) {
    sum += data[i];
  }
  return sum;
}