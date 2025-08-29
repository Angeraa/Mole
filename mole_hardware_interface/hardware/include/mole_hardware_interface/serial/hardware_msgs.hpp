#pragma once

#include <cstdint>
#include <cstddef>

#pragma pack(push, 1)
struct VelocityPacket {
  uint8_t header1 = 0xAA;
  uint8_t header2 = 0xBB;
  uint8_t type = 0x00; // 0x00 for velocity command
  float left_velocity;
  float right_velocity;
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
#pragma pack(pop)

uint8_t calculate_checksum(const uint8_t* data, size_t length) {
  uint8_t sum = 0;
  for (size_t i = 0; i < length; ++i) {
    sum += data[i];
  }
  return sum;
}