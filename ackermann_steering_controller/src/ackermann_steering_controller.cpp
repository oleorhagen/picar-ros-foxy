// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Ole P. Orhagen
 */

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "ackermann_steering_controller/ackermann_steering_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace ackermann_steering_controller
{
using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

AckermannSteeringController::AckermannSteeringController()
: controller_interface::ControllerInterface() {}

controller_interface::return_type
AckermannSteeringController::init(const std::string & controller_name)
{
  // initialize lifecycle node
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::SUCCESS) {
    return ret;
  }

  try {
    auto node = get_node();

    // Velocity control joints for the motors
    node->declare_parameter<std::string>("left_front_wheel_joint");
    node->declare_parameter<std::string>("right_front_wheel_joint");
    node->declare_parameter<std::string>("left_rear_wheel_joint");
    node->declare_parameter<std::string>("right_rear_wheel_joint");

    // Front steer joints
    node->declare_parameter<std::string>("left_front_steer_joint");
    node->declare_parameter<std::string>("right_front_steer_joint");

  return controller_interface::return_type::SUCCESS;
}

InterfaceConfiguration AckermannSteeringController::command_interface_configuration() const
{
  // Ackermann 4WD setup

  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type =
    controller_interface::interface_configuration_type::INDIVIDUAL;

  // Motor command interfaces setup
  command_interfaces_config.names.push_back("left_front_wheel_joint" + "/" + hardware_interface::HW_IF_VELOCITY);
  command_interfaces_config.names.push_back("right_front_wheel_joint" + "/" + hardware_interface::HW_IF_VELOCITY);
  command_interfaces_config.names.push_back("left_rear_wheel_joint" + "/" +
                                            hardware_interface::HW_IF_VELOCITY);
  command_interfaces_config.names.push_back("right_rear_wheel_joint" + "/" +
                                            hardware_interface::HW_IF_VELOCITY);

  // Steer command interfaces setup
  command_interfaces_config.names.push_back("left_front_steer_joint" + "/" +
                                            hardware_interface::HW_IF_POSITION);
  command_interfaces_config.names.push_back("right_front_steer_joint" + "/" +
                                            hardware_interface::HW_IF_POSITION);

  return command_interfaces_config;
}

InterfaceConfiguration AckermannSteeringController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  // Motor states
  conf_names.push_back("left_front_wheel_joint" + "/" + HW_IF_VELOCITY);
  conf_names.push_back("right_front_wheel_joint" + "/" + HW_IF_VELOCITY);
  conf_names.push_back("left_rear_wheel_joint" + "/" + HW_IF_VELOCITY);
  conf_names.push_back("right_rear_wheel_joint" + "/" + HW_IF_VELOCITY);
  // Steer states
  conf_names.push_back("left_front_steer_joint" + "/" + HW_IF_POSITION);
  conf_names.push_back("right_front_steer_joint" + "/" + HW_IF_POSITION);
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}


controller_interface::return_type AckermannSteeringController::update()
{
  auto logger = node_->get_logger();
  auto steering_commands = rt_command_ptr_.readFromRT();
  // No command received yet
  if (!steering_commands || !(*steering_commands)) {
    return controller_interface::return_type::SUCCESS;
  }

  // Extract the steering commands. All other parameters are ignored.
  double linear_command = last_msg->twist.linear.x;
  double angular_command = last_msg->twist.angular.z;

  //
  // Set the wheel velocities
  //

  // TODO - How to set the individual interface values (?)
  todo_left_front_wheel_interface.set_value(linear_command);

  return controller_interface::return_type::SUCCESS;
  }

CallbackReturn AckermannSteeringController::on_configure(const rclcpp_lifecycle::State &)
{
  auto logger = node_->get_logger();

  // Configure motor wheels
  left_front_wheel_joint = node_->get_parameter("left_front_wheel_joint").as_string();
  right_front_wheel_joint =
    node_->get_parameter("right_front_wheel_joint").as_string();
  left_rear_wheel_joint =
    node_->get_parameter("left_rear_wheel_joint").as_string();
  right_rear_wheel_joint =
    node_->get_parameter("right_rear_wheel_joint").as_string();

  // Configure steering wheels
  left_front_steer_joint =
    node_->get_parameter("left_front_steer_joint").as_string();
  right_front_steer_joint =
    node_->get_parameter("right_front_steer_joint").as_string();

  // Create a subscription to /cmd_vel
  node_->create_subscription<geometry_msgs::msg::Twist>(
                                                        DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
                                                        [this](const CmdType::SharedPtr msg) {
                                                          rt_command_ptr_.writeFromNonRT(msg);
                                                        });

  RCLCPP_INFO_STREAM(logger, "configure successful");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AckermannSteeringController::on_activate(const rclcpp_lifecycle::State &)
  {

    subscriber_is_active_ = true;

    // Motor wheel command interface handles
    front_left_wheel = get_wheel_handle_from_name(left_front_wheel_joint);
    front_right_wheel = get_wheel_handle_from_name(right_front_wheel_joint);
    rear_left_wheel = get_wheel_handle_from_name(left_rear_wheel_joint);
    rear_right_wheel = get_wheel_handle_from_name(right_rear_wheel_joint);

    // TODO - Steer wheel handles

    RCLCPP_DEBUG(node_->get_logger(), "Subscriber is now active.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn AckermannSteeringController::on_deactivate(const rclcpp_lifecycle::State &)
  {
    subscriber_is_active_ = false;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn AckermannSteeringController::on_cleanup(const rclcpp_lifecycle::State &)
  {
    if (!reset()) {
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn AckermannSteeringController::on_error(const rclcpp_lifecycle::State &)
  {
    if (!reset()) {
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  bool AckermannSteeringController::reset()
  {
    return true;
  }

  CallbackReturn AckermannSteeringController::on_shutdown(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

  void AckermannSteeringController::halt()
  {
    // TODO
  }

std::shared_ptr<WheelHandle> get_wheel_handle_from_name(const std::string & wheel_joint_name) {

    // Lookup the velocity state interface
    const auto velocity_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&wheel_joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });
    if (velocity_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s velocity state interface not found", wheel_joint_name.c_str());
        return nullptr;
    }

    // Lookup the velocity command interface
    const auto velocity_command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&wheel_joint_name](const hardware_interface::LoanedCommandInterface & interface)
    {
        return interface.get_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });
    if (velocity_command == command_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s velocity command interface not found", wheel_joint_name.c_str());
        return nullptr;
    }

    // Create the wheel instance
    return std::make_shared<WheelHandle>(
        std::ref(*velocity_state),
        std::ref(*velocity_command)
        );

}

}  // namespace diff_drive_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  ackermann_steering_controller::AckermannSteeringController,
  controller_interface::ControllerInterface)
