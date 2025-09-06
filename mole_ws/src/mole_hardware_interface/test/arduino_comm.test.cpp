#include "../hardware/include/mole_hardware_interface/serial/arduino_comm.hpp"
#include "../hardware/include/mole_hardware_interface/serial/libserial_serialport.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <iostream>
#include <thread>
#include <chrono>

TEST(ArduinoCommTest, BasicFunctionality) {
  auto serial_port = std::make_unique<LibSerialPortWrapper>();
  ArduinoComm arduino_comm(std::move(serial_port));

  std::string port = "/dev/ttyUSB0"; // Change as needed
  int baud_rate = 921600;
  arduino_comm.initialize(port, baud_rate);

  ASSERT_TRUE(arduino_comm.get_serial_port()->isOpen()) << "Failed to open serial port.";

  int left_encoder = 0;
  int right_encoder = 0;

  arduino_comm.readEncoders(left_encoder, right_encoder); // Running the comm test code for the esp32 will make the response values be 20 and 40

  std::cout << "Left Encoder: " << left_encoder << ", Right Encoder: " << right_encoder << std::endl;

  std::this_thread::sleep_for(std::chrono::milliseconds(400));

  arduino_comm.writeVelocities(100, 100); // Running the comm test code for the esp32 will make the esp32 led blink twice

  std::this_thread::sleep_for(std::chrono::milliseconds(400));

  arduino_comm.writePID(1, 0, 0, 100); // Running the comm test code for the esp32 will make the esp32 led blink thrice
}

int main() {
  testing::InitGoogleTest();
  return RUN_ALL_TESTS();
}