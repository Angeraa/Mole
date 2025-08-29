#include "include/mole_hardware_interface/mole_diff_system.hpp"

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
    // Initialize hardware positions, velocities, and commands
    hw_positions_.resize(info.joints.size(), 0.0);
    hw_velocities_.resize(info.joints.size(), 0.0);
    hw_commands_.resize(info.joints.size(), 0.0);

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
    for (uint i = 0; i < hw_positions_.size(); i++) {
      hw_positions_[i] = 0.0;
      hw_velocities_[i] = 0.0;
      hw_commands_[i] = 0.0;
    }

    RCLCPP_INFO(rclcpp::get_logger("MoleDiffSystem"), "Configured successfully!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MoleDiffSystem::on_activate(const rclcpp_lifecycle::State &previous_state) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MoleDiffSystem::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MoleDiffSystem::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> MoleDiffSystem::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]
      ));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]
      ));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> MoleDiffSystem::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]
      ));
    }
    
    return command_interfaces;
  }

  hardware_interface::return_type MoleDiffSystem::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
    
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MoleDiffSystem::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
    
    return hardware_interface::return_type::OK;
  }
} // namespace mole_hardware_integration