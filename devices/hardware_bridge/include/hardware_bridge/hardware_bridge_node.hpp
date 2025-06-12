// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
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

#ifndef HARDWARE_BRIDGE__HARDWARE_BRIDGE_NODE_HPP_
#define HARDWARE_BRIDGE__HARDWARE_BRIDGE_NODE_HPP_

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
// #include "tita_robot/tita_robot.hpp"
#include "serial_handle.hpp"

namespace tita_locomotion
{
  constexpr float POS_SCALE = 5215.03f; //  1/32767 * 6.28
  constexpr float VEL_SCALE = 655.34f;  //   1/32767 * 50
  constexpr float TAU_SCALE = 655.34f;  //   1/32767 * 50
  struct Joint
  {
    double position = 0.0f;
    double velocity = 0.0f;
    double effort = 0.0f;

    double effortCommand = 0.0f;
    double positionCommand = 0.0f;
    double velocityCommand = 0.0f;
    double kp = 0.0f;
    double kd = 0.0f;
    std::string name;
  };

  struct InertiaUnit
  {
    std::string name;
    double linear_acceleration[3];
    double angular_velocity[3];
    double orientation[4]; // x y z w
  };

  class HardwareBridge : public hardware_interface::SystemInterface
  {
  public:
    HardwareBridge();
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;
    // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    //     const rclcpp_lifecycle::State & /*previous_state*/) override;
    // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    //     const rclcpp_lifecycle::State & /*previous_state*/) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;
    hardware_interface::return_type write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

  private:
    std::vector<Joint> mJoints;
    InertiaUnit mImu;
    std::shared_ptr<SerialHandle> diablo_joint_sdk_;
    motor_torque_t sendStruct_;
    std::vector<double> joint_direction_ = {-1., -1., 1., -1., -1., 1.};
    std::vector<double> joint_offset_ = {2 * M_PI - 3.256 + 0.1845, 2 * M_PI, 0.,
                                         -3.266 + 0.1845, 0., 0.};
    // std::unique_ptr<tita_robot> robot_;
    bool direct_mode_ = false;
  };
} // namespace tita_locomotion

#endif // HARDWARE_BRIDGE__HARDWARE_BRIDGE_NODE_HPP_
