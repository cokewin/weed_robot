// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef CANOPEN_ROS2_CONTROLLER__CANOPEN_CIA402_CONTROLLER_HPP_
#define CANOPEN_ROS2_CONTROLLER__CANOPEN_CIA402_CONTROLLER_HPP_

#include "canopen_interfaces/srv/co_target_double.hpp"
#include "canopen_ros2_controller/canopen_proxy_controller.hpp"
#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "canopen_ros2_controller/odometry.hpp"
#include "canopen_ros2_controller/speed_limiter.hpp"
#include "canopen_ros2_controller/visibility_control.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "canopen_ros2_controller_parameters.hpp"

static constexpr int kLoopPeriodMS = 100;
static constexpr double kCommandValue = 1.0;

namespace
{
enum Cia402CommandInterfaces
{
  INIT_CMD = CommandInterfaces::LAST_COMMAND_AUX,
  INIT_FBK,
  HALT_CMD,
  HALT_FBK,
  RECOVER_CMD,
  RECOVER_FBK,
  POSITION_MODE_CMD,
  POSITION_MODE_FBK,
  VELOCITY_MODE_CMD,
  VELOCITY_MODE_FBK,
  CYCLIC_VELOCITY_MODE_CMD,
  CYCLIC_VELOCITY_MODE_FBK,
  CYCLIC_POSITION_MODE_CMD,
  CYCLIC_POSITION_MODE_FBK,
  INTERPOLATED_POSITION_MODE_CMD,
  INTERPOLATED_POSITION_MODE_FBK,
};

enum Cia402StateInterfaces
{
  FIRST_STATE = StateInterfaces::LAST_STATE_AUX,
};

}  // namespace
// 前面是canopen通信特有的
namespace canopen_ros2_controller
{

class Cia402DeviceController : public canopen_ros2_controller::CanopenProxyController
{
  using Twist = geometry_msgs::msg::TwistStamped;
public:
  CANOPEN_ROS2_CONTROLLER__VISIBILITY_PUBLIC
  Cia402DeviceController();

  CANOPEN_ROS2_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const;

  CANOPEN_ROS2_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const;

  CANOPEN_ROS2_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period);

  CANOPEN_ROS2_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init();

  CANOPEN_ROS2_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);

  CANOPEN_ROS2_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);

  CANOPEN_ROS2_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state);

  // CANOPEN_ROS2_CONTROLLER__VISIBILITY_PUBLIC
  // controller_interface::CallbackReturn on_cleanup(
  //   const rclcpp_lifecycle::State & previous_state);

  // CANOPEN_ROS2_CONTROLLER__VISIBILITY_PUBLIC
  // controller_interface::CallbackReturn on_error(
  //   const rclcpp_lifecycle::State & previous_state);

  // CANOPEN_ROS2_CONTROLLER__VISIBILITY_PUBLIC
  // controller_interface::CallbackReturn on_shutdown(
  //   const rclcpp_lifecycle::State & previous_state);

protected:
  inline rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr createTriggerSrv(
    const std::string & service, Cia402CommandInterfaces cmd, Cia402CommandInterfaces fbk)
  {
    // define service profile
    auto service_profile = rmw_qos_profile_services_default;
    service_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    service_profile.depth = 1;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv =
      get_node()->create_service<std_srvs::srv::Trigger>(
        service,
        [&, cmd, fbk](
          const std_srvs::srv::Trigger::Request::SharedPtr request,
          std_srvs::srv::Trigger::Response::SharedPtr response)
        {
          command_interfaces_[cmd].set_value(kCommandValue);

          while (std::isnan(command_interfaces_[fbk].get_value()) && rclcpp::ok())
          {
            std::this_thread::sleep_for(std::chrono::milliseconds(kLoopPeriodMS));
          }

          // report success
          response->success = static_cast<bool>(command_interfaces_[fbk].get_value());
          // reset to nan
          command_interfaces_[fbk].set_value(std::numeric_limits<double>::quiet_NaN());
          command_interfaces_[cmd].set_value(std::numeric_limits<double>::quiet_NaN());
        },
        service_profile);

    return srv;
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_init_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_halt_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_recover_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_position_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_torque_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_velocity_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_cyclic_velocity_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_cyclic_position_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_interpolated_position_service_;
  rclcpp::Service<canopen_interfaces::srv::COTargetDouble>::SharedPtr handle_set_target_service_;

  // this is diff config
  struct WheelHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
  };

  const char * feedback_type() const;
  controller_interface::CallbackReturn configure_side(
    const std::string & side, const std::vector<std::string> & wheel_names,
    std::vector<WheelHandle> & registered_handles);

  std::vector<WheelHandle> registered_left_wheel_handles_;
  std::vector<WheelHandle> registered_right_wheel_handles_;

  // Parameters from ROS for diff_drive_controller
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  Odometry odometry_;

  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{500};

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_ = nullptr;

  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ =
    nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    realtime_odometry_transform_publisher_ = nullptr;

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    velocity_command_unstamped_subscriber_ = nullptr;

  realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};

  std::queue<Twist> previous_commands_;  // last two commands

  // speed limiters
  SpeedLimiter limiter_linear_;
  SpeedLimiter limiter_angular_;

  bool publish_limited_velocity_ = false;
  std::shared_ptr<rclcpp::Publisher<Twist>> limited_velocity_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<Twist>> realtime_limited_velocity_publisher_ =
    nullptr;

  rclcpp::Time previous_update_timestamp_{0};

  // publish rate limiter
  double publish_rate_ = 50.0;
  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};

  bool is_halted = false;
  bool use_stamped_vel_ = true;

  bool reset();
  void halt();
};

}  // namespace canopen_ros2_controller

#endif  // CANOPEN_ROS2_CONTROLLER__CANOPEN_CIA402_CONTROLLER_HPP_
