#pragma once

#include <iostream>
#include <stdexcept>


#include "libserial/SerialPort.h"
#include "serialport_iface.hpp"

LibSerial::BaudRate convert_baud_rate(int baud_rate) {
  switch (baud_rate) {
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    default: std::cout << "Error: Unsupported baud rate >>" << baud_rate << ". Defaulting to 57600" << std::endl; 
    return LibSerial::BaudRate::BAUD_57600;
  }
}

class LibSerialPortWrapper : SerialPortIface {
  LibSerial::SerialPort serial_port_;

  public:
    LibSerialPortWrapper() = default;
    ~LibSerialPortWrapper() override {
      if (serial_port_.IsOpen()) {
        serial_port_.Close();
      }
    }

    bool is_open() const override {
      return serial_port_.IsOpen();
    }

    void open(const std::string &port, int baud_rate) override {
      try {
        serial_port_.Open(port);
        serial_port_.SetBaudRate(convert_baud_rate(baud_rate));
      } catch (const LibSerial::OpenFailed &e) {
        throw std::runtime_error(std::string("Error opening serial port: ") + e.what());
      } catch (const LibSerial::AlreadyOpen &e) {
        throw std::runtime_error(std::string("Error: Serial port already open: ") + e.what());
      } catch (const std::exception &e) {
        throw std::runtime_error(std::string("Error initializing serial port: ") + e.what());
      }
    }

    void close() override {
      if (serial_port_.IsOpen()) {
        serial_port_.Close();
      }
    }

    void read(std::vector<uint8_t> &buffer, size_t size, int timeout) override {
      try {
        serial_port_.Read(buffer, size, timeout);
      } catch (const LibSerial::ReadTimeout &e) {
        throw std::runtime_error(std::string("Read timeout: ") + e.what());
      } catch (const std::exception &e) {
        throw std::runtime_error(std::string("Error reading from serial port: ") + e.what());
      }
    }

    void write(const std::vector<uint8_t> &buffer) override {
      try {
        serial_port_.Write(buffer);
      } catch (const std::exception &e) {
        throw std::runtime_error(std::string("Error writing to serial port: ") + e.what());
      }
    }
};