#ifndef MOLE_DIFF_HPP
#define MOLE_DIFF_HPP

#include <memory>
#include <string>
#include <vector>
#include <optional>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "serial/arduino_comm.hpp"
#include "wheel.hpp"

namespace mole_hardware_integration {
  class MoleDiffSystem : public hardware_interface::SystemInterface {
    struct Config {
      std::string left_wheel_name;
      std::string right_wheel_name;
      float loop_rate;
      std::string port;
      int baud_rate;
      int timeout_ms;
      int enc_ticks_per_rev;
      int pid_p = 0;
      int pid_i = 0;
      int pid_d = 0;
      int pid_o = 0;
    };

    std::optional<ArduinoComm> arduino_comm_;
    Config config_;

    Wheel left_wheel_;
    Wheel right_wheel_;
    
    public:
      RCLCPP_SHARED_PTR_DEFINITIONS(MoleDiffSystem)

      hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
      hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
      hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
      hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
      hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  };
} // namespace mole_hardware_integration

#endif