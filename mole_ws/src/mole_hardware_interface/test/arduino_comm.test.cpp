#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <memory>
#include "../hardware/include/mole_hardware_interface/serial/arduino_comm.hpp"
#include "../hardware/include/mole_hardware_interface/serial/libserial_serialport.hpp"

TEST(ArduinoCommTest, BasicFunctionality) {
  auto serial_port = std::make_unique<LibSerialPortWrapper>();
  ArduinoComm arduino_comm(std::move(serial_port));

  std::string port = "/dev/ttyUSB0"; // Change as needed
  int baud_rate = 115200;

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  arduino_comm.initialize(port, baud_rate);

  if (!arduino_comm.get_serial_port()->isOpen()) {
    std::cerr << "Failed to open serial port." << std::endl;
    arduino_comm.close();
    FAIL() << "Failed to open serial port.";
    return;
  }

  int left_encoder = 0;
  int right_encoder = 0;

  try {
    arduino_comm.readEncoders(left_encoder, right_encoder);
    std::cout << "Left Encoder: " << left_encoder << ", Right Encoder: " << right_encoder << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    arduino_comm.writeVelocities(100, 100);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    arduino_comm.writePID(1, 0, 0, 100);
  } catch (const std::exception &e) {
    std::cerr << "Error during ArduinoComm operations: " << e.what() << std::endl;
    arduino_comm.close();
    FAIL() << "Exception: " << e.what();
    return;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  arduino_comm.close();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  ASSERT_FALSE(arduino_comm.get_serial_port()->isOpen()) << "Failed to close serial port.";

  std::cout << "ArduinoComm basic functionality test completed." << std::endl;
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}