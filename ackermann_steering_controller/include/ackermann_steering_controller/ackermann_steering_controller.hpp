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
 * Author: Bence Magyar, Enrique Fernández, Manuel Meraz
 */

#ifndef DIFF_DRIVE_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_
#define DIFF_DRIVE_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "ackermann_steering_controller/odometry.hpp"
#include "ackermann_steering_controller/speed_limiter.hpp"
#include "ackermann_steering_controller/visibility_control.h"
#include "hardware_interface/handle.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"

// #include "ackermann_steering_controller/ackermann_controller_wheel.hpp"


namespace ackermann_steering_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AckermannSteeringController : public controller_interface::ControllerInterface
{
  using Twist = geometry_msgs::msg::TwistStamped;

public:
  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  AckermannSteeringController();

  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  controller_interface::return_type
  init(const std::string & controller_name) override;

  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  controller_interface::return_type update() override;

  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

protected:

  struct WheelHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> state_velocity;
    std::reference_wrapper<const hardware_interface::LoanedCommandInterface> command_velocity;
  };


      bool subscriber_is_active_                                          = false;
      rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;

      realtime_tools::RealtimeBuffer<std::shared_ptr<Twist>> rt_command_ptr_;

      // Motor joint names
      std::string front_left_wheel_joint;
      std::string front_right_wheel_joint;
      std::string rear_left_wheel_joint;
      std::string rear_right_wheel_joint;

      // Steer joint names
      std::string front_left_steer_joint;
      std::string front_right_steer_joint;

      // Command interfaces for the motor controlled wheels
      std::shared_ptr<WheelHandle> front_left_wheel;
      std::shared_ptr<WheelHandle> front_right_wheel;
      std::shared_ptr<WheelHandle> rear_left_wheel;
      std::shared_ptr<WheelHandle> rear_right_wheel;

      // Command interfaces for the front wheel steers
      std::shared_ptr<WheelHandle> front_left_steer_wheel;
      std::shared_ptr<WheelHandle> front_right_steer_wheel;

      bool reset();
      void halt();

  CallbackReturn get_wheel_handle_from_name(const std::string &wheel_joint_name, WheelHandle & handle);
  };
}  // namespace diff_drive_controller
#endif  // DIFF_DRIVE_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_
