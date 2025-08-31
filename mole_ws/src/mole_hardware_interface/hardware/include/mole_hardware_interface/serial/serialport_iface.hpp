#pragma once

#include <vector>
#include <cstdint>
#include <string>

class SerialPortIface {
  public:
    virtual ~SerialPortIface() = default;
    virtual bool isOpen() const = 0;
    virtual void open(const std::string &port, int baud_rate) = 0;
    virtual void close() = 0;
    virtual void read(std::vector<uint8_t> &buffer, size_t size, int timeout) = 0;
    virtual void write(const std::vector<uint8_t> &buffer) = 0;
};