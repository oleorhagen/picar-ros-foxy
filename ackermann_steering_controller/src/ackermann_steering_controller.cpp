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
  : controller_interface::ControllerInterface(),
    rt_command_ptr_(nullptr){}

controller_interface::return_type
AckermannSteeringController::init(const std::string & controller_name)
{
  // initialize lifecycle node
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  try {
    auto node = get_node();

    // Velocity control joints for the motors
    node->declare_parameter<std::string>("left_front_wheel_joint", "left_wheel_joint");
    node->declare_parameter<std::string>("right_front_wheel_joint", "");
    node->declare_parameter<std::string>("left_rear_wheel_joint", "");
    node->declare_parameter<std::string>("right_rear_wheel_joint", "");

    // Front steer joints
    node->declare_parameter<std::string>("left_front_steer_joint", "");
    node->declare_parameter<std::string>("right_front_steer_joint", "");
  }
  catch(const std::exception &e) {
    fprintf(stderr, "Exception thrown during initialization of the Ackermann steering controller. Message: %s\n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

InterfaceConfiguration AckermannSteeringController::command_interface_configuration() const
{
  // Ackermann 4WD setup

  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type =
    controller_interface::interface_configuration_type::INDIVIDUAL;

  // Motor command interfaces setup
  command_interfaces_config.names.push_back(front_left_wheel_joint + "/" + hardware_interface::HW_IF_VELOCITY);
  // command_interfaces_config.names.push_back(front_right_wheel_joint + "/" + hardware_interface::HW_IF_VELOCITY);
  // command_interfaces_config.names.push_back(rear_left_wheel_joint + "/" +
  //                                           hardware_interface::HW_IF_VELOCITY);
  // command_interfaces_config.names.push_back(rear_right_wheel_joint + "/" +
  //                                           hardware_interface::HW_IF_VELOCITY);

  // // Steer command interfaces setup
  // command_interfaces_config.names.push_back(front_left_steer_joint + "/" +
  //                                           hardware_interface::HW_IF_POSITION);
  // command_interfaces_config.names.push_back(front_right_steer_joint + "/" +
  //                                           hardware_interface::HW_IF_POSITION);

  return command_interfaces_config;
}

InterfaceConfiguration AckermannSteeringController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type =
    controller_interface::interface_configuration_type::INDIVIDUAL;
  // Motor states
  state_interfaces_config.names.push_back(front_left_wheel_joint + "/" + HW_IF_VELOCITY);
  // conf_names.push_back(front_right_wheel_joint + "/" + HW_IF_VELOCITY);
  // conf_names.push_back(rear_left_wheel_joint + "/" + HW_IF_VELOCITY);
  // conf_names.push_back(rear_right_wheel_joint + "/" + HW_IF_VELOCITY);
  // Steer states
  // conf_names.push_back(front_left_steer_joint + "/" + HW_IF_POSITION);
  // conf_names.push_back(front_right_steer_joint + "/" + HW_IF_POSITION);
  return state_interfaces_config;
}


controller_interface::return_type AckermannSteeringController::update()
{
  auto logger = node_->get_logger();
  auto steering_commands = rt_command_ptr_.readFromNonRT();
  // Set dummy speed of 1.0 to test
  front_left_wheel->command_velocity.get().set_value(1.0);
  // No command received yet
  if (!steering_commands || !(*steering_commands)) {
    RCLCPP_INFO_STREAM(logger, "\n--------------------------------------------------------------------------------No steering command received....\n--------------------------------------------------------------------------------\n");
    return controller_interface::return_type::OK;
  }

  // Extract the steering commands. All other parameters are ignored.
  // double linear_command = last_msg->twist.linear.x;
  // double angular_command = last_msg->twist.angular.z;

  //
  // Set the wheel velocities
  //

  RCLCPP_INFO(node_->get_logger(), "Setting velocity to: %f\n", (*steering_commands)->twist.linear.x);
  front_left_wheel->command_velocity.get().set_value((*steering_commands)->twist.linear.x);

  // TODO - How to set the individual interface values (?)
  // front_left_wheel.command_velocity.set_value(0.0);

  return controller_interface::return_type::OK;
  }

CallbackReturn AckermannSteeringController::on_configure(const rclcpp_lifecycle::State &)
{
  auto logger = node_->get_logger();

  // Configure motor wheels
  front_left_wheel_joint = node_->get_parameter("left_front_wheel_joint").as_string();
  front_right_wheel_joint =
    node_->get_parameter("right_front_wheel_joint").as_string();
  rear_left_wheel_joint =
    node_->get_parameter("left_rear_wheel_joint").as_string();
  rear_right_wheel_joint =
    node_->get_parameter("right_rear_wheel_joint").as_string();

  // Configure steering wheels
  front_left_steer_joint =
    node_->get_parameter("left_front_steer_joint").as_string();
  front_right_steer_joint =
    node_->get_parameter("right_front_steer_joint").as_string();

  // Create a subscription to /cmd_vel
  velocity_command_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
         "cmd_vel", rclcpp::SystemDefaultsQoS(),
         [this](const Twist::SharedPtr msg) -> void {
           RCLCPP_INFO(node_->get_logger(), "Received Twist message: %f", msg->twist.linear.x);
           rt_command_ptr_.writeFromNonRT(msg);
           RCLCPP_INFO(node_->get_logger(), "Wrote message to the");
         });

  RCLCPP_INFO_STREAM(logger, "configure successful");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AckermannSteeringController::on_activate(const rclcpp_lifecycle::State &)
  {
      auto logger = node_->get_logger();

      subscriber_is_active_ = true;

      RCLCPP_INFO(logger, "Activating the Ackermann steering controller...");

      // Motor wheel command interface handles
      // TODO - Only testing with one wheel for now
      // front_left_wheel = get_wheel_handle_from_name(front_left_wheel_joint, &front_left_wheel);

      // Lookup the velocity state interface
      for (const auto &state_interface : state_interfaces_)
        {
          RCLCPP_INFO(logger, "Found interface: %s", state_interface.get_interface_name());
          if (state_interface.get_name() == front_left_wheel_joint &&
              state_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
            {
              RCLCPP_DEBUG(logger, "Found interface: %s", state_interface.get_name());

              // Command interface
              for (auto &command_interface : command_interfaces_)
                {

                  RCLCPP_INFO(logger, "Found interface: %s", command_interface.get_interface_name());
                  if (command_interface.get_name() == front_left_wheel_joint &&
                      command_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
                    {
                      RCLCPP_DEBUG(logger, "Found interface: %s", command_interface.get_name());
                      front_left_wheel = std::make_shared<WheelHandle>(WheelHandle{
                          std::ref(state_interface),
                          std::ref(command_interface),
                        });
                    }
                }
            }
        }



    // TODO - Steer wheel handles

    RCLCPP_INFO(node_->get_logger(), "Subscriber is now active.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn AckermannSteeringController::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Deactivating...");
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
    RCLCPP_INFO_STREAM(node_->get_logger(), "On_error");
    if (!reset()) {
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  bool AckermannSteeringController::reset()
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Resetting...");
    velocity_command_subscriber_.reset();
    return true;
  }

  CallbackReturn AckermannSteeringController::on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Shutting down...");
    return CallbackReturn::SUCCESS;
  }

  void AckermannSteeringController::halt()
  {
    // TODO
    RCLCPP_INFO_STREAM(node_->get_logger(), "Halting...");
  }

// CallbackReturn AckermannSteeringController::get_wheel_handle_from_name(const std::string & wheel_joint_name, std::shared_ptr<WheelHandle> & handle) {

//     // Lookup the velocity state interface
//     const auto velocity_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&wheel_joint_name](const hardware_interface::LoanedStateInterface & interface)
//     {
//         return interface.get_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
//     });
//     if (velocity_state == state_interfaces_.cend()) {
//         RCLCPP_ERROR(get_node()->get_logger(), "%s velocity state interface not found", wheel_joint_name.c_str());
//         return nullptr;
//     }

//     // Lookup the velocity command interface
//     const auto velocity_command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&wheel_joint_name](const hardware_interface::LoanedCommandInterface & interface)
//     {
//         return interface.get_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
//     });
//     if (velocity_command == command_interfaces_.end()) {
//         RCLCPP_ERROR(get_node()->get_logger(), "%s velocity command interface not found", wheel_joint_name.c_str());
//         return nullptr;
//     }

//     // Create the wheel instance
//     // return std::make_shared<ackermann_steering_controller::AckermannSteeringController::WheelHandle>(
//     //     std::ref(*velocity_state),
//     //     std::ref(*velocity_command)
//     //     );
//     // return std::make_shared<ackermann_steering_controller::AckermannSteeringController::WheelHandle>WheelHandle{
//     //   std::ref(*velocity_state),
//     //   std::ref(*velocity_command),
//     // };
//     // std::ref<WheelHandle>WheelHandle{
//     //   std::ref(*velocity_state),
//     //     std::ref(*velocity_command),
//     //     };
//     // return nullptr;
//     return CallbackReturn::SUCCESS;
// }

}  // namespace ackermann_steering_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  ackermann_steering_controller::AckermannSteeringController,
  controller_interface::ControllerInterface)
