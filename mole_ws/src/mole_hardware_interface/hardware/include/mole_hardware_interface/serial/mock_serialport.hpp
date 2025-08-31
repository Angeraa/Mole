#pragma once

#include <stdexcept>

#include "serialport_iface.hpp"

class MockSerialPort : public SerialPortIface {
  bool open_ = false;
  std::vector<uint8_t> read_buffer_;
  std::vector<uint8_t> write_buffer_;
  
  public:
    MockSerialPort() = default;
    ~MockSerialPort() override = default;

    bool isOpen() const override {
      return open_;
    }

    void open(const std::string &port, int baud_rate) override {
      open_ = true;
    }

    void close() override {
      open_ = false;
    }

    void read(std::vector<uint8_t> &buffer, size_t size, int timeout) override {
      if (read_buffer_.size() < size) {
        throw std::runtime_error("Not enough data in read buffer");
      }
      buffer.insert(buffer.end(), read_buffer_.begin(), read_buffer_.begin() + size);
      read_buffer_.erase(read_buffer_.begin(), read_buffer_.begin() + size);
    }

    void write(const std::vector<uint8_t> &buffer) override {
      write_buffer_.insert(write_buffer_.end(), buffer.begin(), buffer.end());
    }

    // Helper methods for testing
    void set_read_data(const std::vector<uint8_t> &data) {
      read_buffer_ = data;
    }

    const std::vector<uint8_t> &get_written_data() const {
      return write_buffer_;
    }
};