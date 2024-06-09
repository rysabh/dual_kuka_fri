#ifndef LBR_ROS2_CONTROL__LBR_JOINT_POSITION_COMMAND_CONTROLLER_HPP_
#define LBR_ROS2_CONTROL__LBR_JOINT_POSITION_COMMAND_CONTROLLER_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <stdexcept>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.h"

#include "friLBRState.h"

#include "lbr_fri_idl/msg/lbr_joint_position_command.hpp"

namespace lbr_ros2_control {
class LBRJointPositionCommandController : public controller_interface::ControllerInterface {
public:
  LBRJointPositionCommandController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::return_type update(const rclcpp::Time &time,
                                           const rclcpp::Duration &period) override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

protected:
  std::array<std::string, KUKA::FRI::LBRState::NUMBER_OF_JOINTS> joint_names_ = {
      "A1", "A2", "A3", "A4", "A5", "A6", "A7"};

  realtime_tools::RealtimeBuffer<lbr_fri_idl::msg::LBRJointPositionCommand::SharedPtr>
      rt_lbr_joint_position_command_ptr_;
  rclcpp::Subscription<lbr_fri_idl::msg::LBRJointPositionCommand>::SharedPtr
      lbr_joint_position_command_subscription_ptr_;
};
} // end of namespace lbr_ros2_control
#endif // LBR_ROS2_CONTROL__LBR_JOINT_POSITION_COMMAND_CONTROLLER_HPP_
