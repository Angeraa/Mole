#pragma once

#include <sstream>
#include <iostream>
#include <cstring>
#include <memory>

#include "hardware_msgs.hpp"
#include "serialport_iface.hpp"



class ArduinoComm {
  std::unique_ptr<SerialPortIface> serial_port_;
  int timeout_ms_ = 100;
  public:
    ArduinoComm(std::unique_ptr<SerialPortIface> serial_port) : serial_port_(std::move(serial_port)) {}

    void initialize(const std::string &port, int baud_rate, int timeout_ms = 100) {
      timeout_ms_ = timeout_ms;
      try {
        serial_port_->open(port, baud_rate);
      } catch (const std::exception &e) {
        std::cerr << "Error initializing serial port: " << e.what() << std::endl;
      }
    }

    void close() {
      serial_port_->close();
    }

    ~ArduinoComm() {
      close();
    }

    SerialPortIface *get_serial_port() {
      return serial_port_.get();
    }

    void readEncoders(int &val_1, int &val_2) {
      EncoderPacket packet;
      std::vector<uint8_t> buffer(sizeof(EncoderPacket));
      try {
        serial_port_->write(std::vector<uint8_t>{0xAA, 0xBB, 0x02, 0x00}); // Request encoder data
        serial_port_->read(buffer, buffer.size(), timeout_ms_);
        std::memcpy(&packet, buffer.data(), sizeof(EncoderPacket));
        // Validate headers
        if (packet.header1 != 0xAA || packet.header2 != 0xBB || packet.type != 0x01) {
          std::cerr << "Invalid packet header or type" << std::endl;
          return;
        }
        // Validate checksum
        uint8_t calculated_checksum = calculate_checksum((uint8_t *)&packet, sizeof(EncoderPacket) - 1);
        if (calculated_checksum != packet.checksum) {
          std::cerr << "Checksum mismatch" << std::endl;
          return;
        }
        val_1 = packet.left_encoder;
        val_2 = packet.right_encoder;
      } catch (const std::exception &e) {
        std::cerr << "Error reading from serial port: " << e.what() << std::endl;
      }
    }

    void writeVelocities(int vel_1, int vel_2) {
      VelocityPacket packet;
      packet.left_velocity = vel_1;
      packet.right_velocity = vel_2;
      packet.checksum = calculate_checksum((uint8_t *)&packet, sizeof(VelocityPacket) - 1);

      std::vector<uint8_t> buffer(
        reinterpret_cast<uint8_t*>(&packet),
        reinterpret_cast<uint8_t*>(&packet) + sizeof(VelocityPacket)
      );

      try {
        serial_port_->write(buffer);
      } catch (const std::exception &e) {
        std::cerr << "Error writing to serial port: " << e.what() << std::endl;
      }
    }

    void writePID(int p, int i, int d, int o) {
      PIDPacket packet;
      packet.p = p;
      packet.i = i;
      packet.d = d;
      packet.o = o;
      packet.checksum = calculate_checksum((uint8_t *)&packet, sizeof(PIDPacket) - 1);

      std::vector<uint8_t> buffer(
        reinterpret_cast<uint8_t*>(&packet),
        reinterpret_cast<uint8_t*>(&packet) + sizeof(PIDPacket)
      );

      try {
        serial_port_->write(buffer);
      } catch (const std::exception &e) {
        std::cerr << "Error writing to serial port: " << e.what() << std::endl;
      }
    }
};