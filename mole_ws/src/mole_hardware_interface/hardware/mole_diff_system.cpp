#include "include/mole_hardware_interface/mole_diff_system.hpp"
#include "serial/arduino_comm.hpp"
#include "serial/libserial_serialport.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <iomanip>
#include <sstream>
#include <cstddef>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mole_hardware_integration {

  hardware_interface::CallbackReturn MoleDiffSystem::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Parse config parameters
    config_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    config_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    config_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    config_.port = info_.hardware_parameters["port"];
    config_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    config_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    config_.enc_ticks_per_rev = std::stoi(info_.hardware_parameters["enc_ticks_per_rev"]);
    config_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    config_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    config_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    config_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);

    // Init Wheels
    left_wheel_.initialize(config_.left_wheel_name, config_.enc_ticks_per_rev);
    right_wheel_.initialize(config_.right_wheel_name, config_.enc_ticks_per_rev);

    // Create Comms
    arduino_comm_.emplace(std::make_unique<LibSerialPortWrapper>());

    for (const hardware_interface::ComponentInfo &joint : info.joints) {
      if (joint.command_interfaces.size() != 1) {
        RCLCPP_FATAL(rclcpp::get_logger("MoleDiffSystem"),
          "Joint '%s' must have exactly one command interface",
          joint.name.c_str()
        );
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
        RCLCPP_FATAL(rclcpp::get_logger("MoleDiffSystem"),
          "Joint '%s' command interface must be '%s', but is '%s'",
          joint.name.c_str(),
          hardware_interface::HW_IF_POSITION,
          joint.command_interfaces[0].name.c_str()
        );
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2) {
        RCLCPP_FATAL(rclcpp::get_logger("MoleDiffSystem"),
          "Joint '%s' must have exactly two state interfaces",
          joint.name.c_str()
        );
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_FATAL(rclcpp::get_logger("MoleDiffSystem"),
          "Joint '%s' state interfaces must be '%s' and '%s', but are '%s' and '%s'",
          joint.name.c_str(),
          hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY,
          joint.state_interfaces[0].name.c_str(),
          joint.state_interfaces[1].name.c_str()
        );
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MoleDiffSystem::on_configure(const rclcpp_lifecycle::State &previous_state) {
    RCLCPP_INFO(rclcpp::get_logger("MoleDiffSystem"), "Configuring...");
    if(arduino_comm_.has_value()) {
      arduino_comm_.value().initialize(config_.port, config_.baud_rate, config_.timeout_ms);
      RCLCPP_INFO(rclcpp::get_logger("MoleDiffSystem"), "Configured successfully!");
      return hardware_interface::CallbackReturn::SUCCESS;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("MoleDiffSystem"), "Arduino communication interface not initialized!");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  hardware_interface::CallbackReturn MoleDiffSystem::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCLCPP_INFO(rclcpp::get_logger("MoleDiffSystem"), "Activating...");
    if(arduino_comm_.has_value()) {
      arduino_comm_.value().writePID(config_.pid_p, config_.pid_i, config_.pid_d, config_.pid_o);
      RCLCPP_INFO(rclcpp::get_logger("MoleDiffSystem"), "Activated successfully!");
      return hardware_interface::CallbackReturn::SUCCESS;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("MoleDiffSystem"), "Arduino communication interface not initialized!");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  hardware_interface::CallbackReturn MoleDiffSystem::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    RCLCPP_INFO(rclcpp::get_logger("MoleDiffSystem"), "Cleaning up...");
    if(arduino_comm_.has_value()) arduino_comm_.value().close();
    RCLCPP_INFO(rclcpp::get_logger("MoleDiffSystem"), "Cleaned up successfully!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> MoleDiffSystem::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Left wheel
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      left_wheel_.name, hardware_interface::HW_IF_POSITION, &left_wheel_.pos
    ));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.vel
    ));

    // Right wheel
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      right_wheel_.name, hardware_interface::HW_IF_POSITION, &right_wheel_.pos
    ));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.vel
    ));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> MoleDiffSystem::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Left wheel
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      left_wheel_.name, hardware_interface::HW_IF_POSITION, &left_wheel_.cmd
    ));

    // Right wheel
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      right_wheel_.name, hardware_interface::HW_IF_POSITION, &right_wheel_.cmd
    ));

    return command_interfaces;
  }

  hardware_interface::return_type MoleDiffSystem::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
    if (!arduino_comm_.has_value()) {
      RCLCPP_ERROR(rclcpp::get_logger("MoleDiffSystem"), "Arduino communication interface not initialized!");
      return hardware_interface::return_type::ERROR;
    }

    arduino_comm_.value().readEncoders(left_wheel_.enc, right_wheel_.enc);

    double delta_time = period.seconds();

    double prev_pos = left_wheel_.pos;
    left_wheel_.pos = left_wheel_.calcEncAngle();
    left_wheel_.vel = (left_wheel_.pos - prev_pos) / delta_time;

    prev_pos = right_wheel_.pos;
    right_wheel_.pos = right_wheel_.calcEncAngle();
    right_wheel_.vel = (right_wheel_.pos - prev_pos) / delta_time;

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MoleDiffSystem::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
    if (!arduino_comm_.has_value()) {
      RCLCPP_ERROR(rclcpp::get_logger("MoleDiffSystem"), "Arduino communication interface not initialized!");
      return hardware_interface::return_type::ERROR;
    }
    int motor_l_ticks_per_loop = left_wheel_.cmd / left_wheel_.rads_per_tick / config_.loop_rate;
    int motor_r_ticks_per_loop = right_wheel_.cmd / right_wheel_.rads_per_tick / config_.loop_rate;

    arduino_comm_.value().writeVelocities(motor_l_ticks_per_loop, motor_r_ticks_per_loop);
    return hardware_interface::return_type::OK;
  }
} // namespace mole_hardware_integration