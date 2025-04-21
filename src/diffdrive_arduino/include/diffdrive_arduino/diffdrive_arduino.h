#ifndef DIFFDRIVE_ARDUINO_REAL_ROBOT_H
#define DIFFDRIVE_ARDUINO_REAL_ROBOT_H

#include <cstring>
#include "rclcpp/rclcpp.hpp"

// Include all necessary hardware_interface headers
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "config.h"
#include "wheel.h"
#include "arduino_comms.h"

using hardware_interface::return_type;

class DiffDriveArduino : public hardware_interface::SystemInterface
{
public:
  DiffDriveArduino();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Hardware info
  hardware_interface::HardwareInfo info_;
  
  Config cfg_;
  ArduinoComms arduino_;

  Wheel l_wheel_;
  Wheel r_wheel_;

  rclcpp::Logger logger_;

  std::chrono::time_point<std::chrono::system_clock> time_;
};

#endif // DIFFDRIVE_ARDUINO_REAL_ROBOT_H