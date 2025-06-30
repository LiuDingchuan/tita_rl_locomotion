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

#include "hardware_bridge/hardware_bridge_node.hpp"

#include <pthread.h>
#include <sched.h>
#include <chrono>
#include <iostream>

#include "pluginlib/class_list_macros.hpp"

namespace tita_locomotion
{
  HardwareBridge::HardwareBridge() {}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareBridge::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    auto ctrl_mode = info_.hardware_parameters["ctrl_mode"];
    // direct_mode_ = false, when ctrl_mode == mcu
    direct_mode_ = ctrl_mode.compare("mcu") == 0 ? false : true;

    diablo_joint_sdk_ = std::make_shared<SerialHandle>();
    diablo_joint_sdk_->serial_init("/dev/ttyUSB0");
    std::cout << "serial init------------------------ " << std::endl;

    for (hardware_interface::ComponentInfo component : info.joints)
    {
      Joint joint;
      joint.name = component.name;
      mJoints.push_back(joint);
    }
    for (hardware_interface::ComponentInfo component : info.sensors)
    {
      std::string sensor_name = component.name;
      mImu.name = sensor_name;
    }
    diablo_joint_sdk_->start_joint_sdk();

    // robot_ = std::make_unique<tita_robot>(mJoints.size());
    // if (
    //     hardware_interface::SystemInterface::on_init(info) !=
    //     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
    // {
    //   return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    // }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  /**
   * @brief: 这里是通过ROS2的SystemInterface接口导出状态接口和命令接口
   *         这些接口会被控制器使用来获取关节状态和发送命令
   *         这里的状态接口包括关节位置、速度、力矩等信息
   *          主要是通过将地址导出的方式进行的
   * @author: Dandelion
   * @Date: 2025-06-16 18:19:29
   * @return {*}
   */
  std::vector<hardware_interface::StateInterface> HardwareBridge::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (Joint &joint : mJoints)
    {
      interfaces.emplace_back(hardware_interface::StateInterface(
          joint.name, hardware_interface::HW_IF_POSITION, &(joint.position)));
      interfaces.emplace_back(hardware_interface::StateInterface(
          joint.name, hardware_interface::HW_IF_VELOCITY, &(joint.velocity)));
      interfaces.emplace_back(hardware_interface::StateInterface(
          joint.name, hardware_interface::HW_IF_EFFORT, &(joint.effort)));
    }

    for (hardware_interface::ComponentInfo component : info_.sensors)
    {
      if (component.name == mImu.name)
      {
        for (uint i = 0; i < 4; i++)
        {
          interfaces.emplace_back(hardware_interface::StateInterface(
              component.name, component.state_interfaces[i].name, &mImu.orientation[i]));
        }
        for (uint i = 0; i < 3; i++)
        {
          interfaces.emplace_back(hardware_interface::StateInterface(
              component.name, component.state_interfaces[i + 4].name, &mImu.angular_velocity[i]));
        }
        for (uint i = 0; i < 3; i++)
        {
          interfaces.emplace_back(hardware_interface::StateInterface(
              component.name, component.state_interfaces[i + 7].name, &mImu.linear_acceleration[i]));
        }
      }
    }
    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface> HardwareBridge::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (Joint &joint : mJoints)
    {
      interfaces.emplace_back(hardware_interface::CommandInterface(
          joint.name, hardware_interface::HW_IF_POSITION, &(joint.positionCommand)));
      interfaces.emplace_back(hardware_interface::CommandInterface(
          joint.name, hardware_interface::HW_IF_VELOCITY, &(joint.velocityCommand)));
      interfaces.emplace_back(hardware_interface::CommandInterface(
          joint.name, hardware_interface::HW_IF_EFFORT, &(joint.effortCommand)));
      interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, "kp", &(joint.kp)));
      interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, "kd", &(joint.kd)));
    }
    return interfaces;
  }

  // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  // HardwareBridge::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  // {
  //   if (direct_mode_)
  //   {
  //     robot_->set_motors_sdk(direct_mode_);
  //   }
  //   return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  // }

  // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  // HardwareBridge::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  // {
  //   return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  // }

  hardware_interface::return_type HardwareBridge::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    static uint16_t last_receive_cnt = 0; // 上一次接收到的包计数
    uart_packet_t lastest_buffer;
    diablo_joint_sdk_->get_latest_packet(lastest_buffer);
    auto diablo_info = &lastest_buffer;
    if (diablo_info->frame_cnt != last_receive_cnt)
    {
      // static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
      // auto now = std::chrono::steady_clock::now();
      // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
      // std::cout << "Time since last packet: " << duration << " us " << " read_cnt: " << diablo_info->frame_cnt << std::endl;
      // last_time = now;

      last_receive_cnt = diablo_info->frame_cnt;
      auto diablo_joints = std::vector<motor_msgs_t>{diablo_info->left_hip, diablo_info->left_knee, diablo_info->left_wheel, diablo_info->right_hip, diablo_info->right_knee, diablo_info->right_wheel};
      std::vector<double> joint_pos, joint_vel, joint_tau;
      for (const auto &diablo_joint : diablo_joints)
      {
        joint_pos.push_back(diablo_joint.pos);
        joint_vel.push_back(diablo_joint.vel);
        joint_tau.push_back(diablo_joint.torque);
      }
      for (size_t id = 0; id < mJoints.size(); id++)
      {
        // data acquire from encoder
        mJoints[id].position = joint_direction_[id] * joint_pos[id] / POS_SCALE - joint_offset_[id];
        mJoints[id].velocity = joint_direction_[id] * joint_vel[id] / VEL_SCALE;
        mJoints[id].effort = joint_direction_[id] * joint_tau[id] / TAU_SCALE;
        if (id == 1 || id == 4)
        {
          if (mJoints[id].position < -3)
          {
            mJoints[id].position += 2 * M_PI;
          }
        }
      }
      // closed loop model to urdf open loop model
      // mJoints[1].position = -mJoints[1].position;
      // mJoints[1].velocity = -mJoints[1].velocity;
      // mJoints[4].position = -mJoints[4].position;
      // mJoints[4].velocity = -mJoints[4].velocity;

      // mJoints[0].position = mJoints[0].position - mJoints[1].position;
      // mJoints[0].velocity = mJoints[0].velocity - mJoints[1].velocity;
      // mJoints[3].position = mJoints[3].position - mJoints[4].position;
      // mJoints[3].velocity = mJoints[3].velocity - mJoints[4].velocity;

      mImu.linear_acceleration[0] = diablo_info->accl.x / 1638.5f * 9.81f;
      mImu.linear_acceleration[1] = diablo_info->accl.y / 1638.5f * 9.81f;
      mImu.linear_acceleration[2] = diablo_info->accl.z / 1638.5f * 9.81f;

      mImu.angular_velocity[0] = diablo_info->gyro.x / 327.67f;
      mImu.angular_velocity[1] = diablo_info->gyro.y / 327.67f;
      mImu.angular_velocity[2] = diablo_info->gyro.z / 327.67f;

      mImu.orientation[0] = diablo_info->orientation.x / 32767.f;
      mImu.orientation[1] = diablo_info->orientation.y / 32767.f,
      mImu.orientation[2] = diablo_info->orientation.z / 32767.f,
      mImu.orientation[3] = diablo_info->orientation.w / 32767.f;
    }
    else
    {
      return hardware_interface::return_type::OK;
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type HardwareBridge::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // std::cout << "write " << std::endl ;
    float motorCmd[6] = {0, 0, 0, 0, 0, 0};
    if (direct_mode_)
    {
      std::vector<double> cmd_torque;
      for (size_t id = 0; id < 6; id++)
      {
        // motorCmd[id] = static_cast<float>(mJoints[id].effortCommand +
        //                                   mJoints[id].kp * (mJoints[id].positionCommand - mJoints[id].position) +
        //                                   mJoints[id].kd * (mJoints[id].velocityCommand - mJoints[id].velocity));
        motorCmd[id] = static_cast<float>(mJoints[id].effortCommand);
      }
    }
    // std::cout << "effortcommand" <<  << std::endl;
    // urdf open loop model to closed loop model
    // motorCmd[1] = -motorCmd[1];
    // motorCmd[4] = -motorCmd[4];

    // motorCmd[2] = 0;
    // motorCmd[5] = 0;
    for (size_t id = 0; id < 6; ++id)
    {
      motorCmd[id] = joint_direction_[id] * motorCmd[id];
    }
    diablo_joint_sdk_->create_package(motorCmd, sendStruct_);
    diablo_joint_sdk_->send_commond(sendStruct_);
    return hardware_interface::return_type::OK;
  }

} // namespace tita_locomotion
PLUGINLIB_EXPORT_CLASS(tita_locomotion::HardwareBridge, hardware_interface::SystemInterface)
