#pragma once

#include <sstream>
#include <iostream>
#include <cstring>

#include "mole_hardware_interface/serial/hardware_msgs.hpp"
#include "serialport_iface.hpp"



class ArduinoComm {
  SerialPortIface &serial_port_;
  public:
    ArduinoComm(SerialPortIface &serial_port) : serial_port_(serial_port) {}

    void initialize(const std::string &port, int baud_rate) {
      try {
        serial_port_.open(port, baud_rate);
      } catch (const std::exception &e) {
        std::cerr << "Error initializing serial port: " << e.what() << std::endl;
      }
    }

    ~ArduinoComm() {
      if (serial_port_.is_open()) {
        serial_port_.close();
      }
    }

    SerialPortIface &get_serial_port() {
      return serial_port_;
    }

    void read_encoders(int &val_1, int &val_2) {
      EncoderPacket packet;
      std::vector<uint8_t> buffer(sizeof(EncoderPacket));
      try {
        serial_port_.read(buffer, buffer.size(), 1000);
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

    void write_velocities(float vel_1, float vel_2) {
      VelocityPacket packet;
      packet.left_velocity = vel_1;
      packet.right_velocity = vel_2;
      packet.checksum = calculate_checksum((uint8_t *)&packet, sizeof(VelocityPacket) - 1);

      std::vector<uint8_t> buffer(
        reinterpret_cast<uint8_t*>(&packet),
        reinterpret_cast<uint8_t*>(&packet) + sizeof(VelocityPacket)
      );

      try {
        serial_port_.write(buffer);
      } catch (const std::exception &e) {
        std::cerr << "Error writing to serial port: " << e.what() << std::endl;
      }
    }
};