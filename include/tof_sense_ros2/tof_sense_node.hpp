// Copyright 2021 Borong Yuan
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

#pragma once

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "serial_driver/serial_driver.hpp"
#include "tof_sense_ros2/visibility_control.hpp"

namespace lc = rclcpp_lifecycle;
using LNI = lc::node_interfaces::LifecycleNodeInterface;
using namespace drivers::serial_driver;

namespace tof_sense
{
class TOFSenseNode final : public lc::LifecycleNode
{
public:
  TOF_SENSE_ROS2_PUBLIC explicit TOFSenseNode(const rclcpp::NodeOptions& options);

  TOF_SENSE_ROS2_PUBLIC ~TOFSenseNode();

  TOF_SENSE_ROS2_PUBLIC LNI::CallbackReturn on_configure(const lc::State& state) override;

  TOF_SENSE_ROS2_PUBLIC LNI::CallbackReturn on_activate(const lc::State& state) override;

  TOF_SENSE_ROS2_PUBLIC LNI::CallbackReturn on_deactivate(const lc::State& state) override;

  TOF_SENSE_ROS2_PUBLIC LNI::CallbackReturn on_cleanup(const lc::State& state) override;

  TOF_SENSE_ROS2_PUBLIC LNI::CallbackReturn on_shutdown(const lc::State& state) override;

  TOF_SENSE_ROS2_PUBLIC void receive_callback(const std::vector<uint8_t>& buffer);

private:
  std::string device_name_{};
  uint32_t baud_rate_, num_sensors_;
  float field_of_view_, min_range_, max_range_;

  std::unique_ptr<IoContext> ctx_{};
  std::unique_ptr<SerialPortConfig> config_;
  std::unique_ptr<SerialDriver> tof_sense;

  std::vector<lc::LifecyclePublisher<sensor_msgs::msg::Range>::SharedPtr> range_pubs;
};
}  // namespace tof_sense
