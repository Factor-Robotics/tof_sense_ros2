// Copyright 2021 Factor Robotics
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

#include "tof_sense_ros2/tof_sense_node.hpp"

namespace tof_sense
{
TOFSenseNode::TOFSenseNode(const rclcpp::NodeOptions& options)
  : lc::LifecycleNode("tof_sense_node", options), ctx_{ new IoContext(1) }, tof_sense{ new SerialDriver(*ctx_) }
{
  device_name_ = declare_parameter<std::string>("device_name", "/dev/ttyS0");
  baud_rate_ = declare_parameter<int>("baud_rate", 115200);
  num_sensors_ = declare_parameter<int>("num_sensors", 1);
  field_of_view_ = declare_parameter<float>("field_of_view", 0.4712389);
  min_range_ = declare_parameter<float>("min_range", 0.01);
  max_range_ = declare_parameter<float>("max_range", 5.00);
  config_ = std::make_unique<SerialPortConfig>(baud_rate_, FlowControl::NONE, Parity::NONE, StopBits::ONE);
}

TOFSenseNode::~TOFSenseNode()
{
  if (ctx_)
  {
    ctx_->waitForExit();
  }
}

LNI::CallbackReturn TOFSenseNode::on_configure(const lc::State&)
{
  try
  {
    tof_sense->init_port(device_name_, *config_);
    if (!tof_sense->port()->is_open())
    {
      tof_sense->port()->open();
      tof_sense->port()->async_receive(std::bind(&TOFSenseNode::receive_callback, this, std::placeholders::_1));
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    return LNI::CallbackReturn::FAILURE;
  }

  for (size_t i = 0; i < num_sensors_; i++)
  {
    range_pubs.push_back(
        this->create_publisher<sensor_msgs::msg::Range>("range" + std::to_string(i), rclcpp::SensorDataQoS()));
  }

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn TOFSenseNode::on_activate(const lc::State&)
{
  for (size_t i = 0; i < num_sensors_; i++)
  {
    range_pubs[i]->on_activate();
  }
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn TOFSenseNode::on_deactivate(const lc::State&)
{
  for (size_t i = 0; i < num_sensors_; i++)
  {
    range_pubs[i]->on_deactivate();
  }
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn TOFSenseNode::on_cleanup(const lc::State&)
{
  tof_sense->port()->close();
  for (size_t i = 0; i < num_sensors_; i++)
  {
    range_pubs[i].reset();
  }
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn TOFSenseNode::on_shutdown(const lc::State&)
{
  tof_sense->port()->close();
  for (size_t i = 0; i < num_sensors_; i++)
  {
    range_pubs[i].reset();
  }
  return LNI::CallbackReturn::SUCCESS;
}

void TOFSenseNode::receive_callback(const std::vector<uint8_t>& buffer)
{
  sensor_msgs::msg::Range range;
  range.header.stamp = this->now();
  if (buffer[0] == 0x57 && buffer[1] == 0 && buffer[2] == 0xff)
  {
    range.header.frame_id = "range" + std::to_string(buffer[3]);
    range.radiation_type = sensor_msgs::msg::Range::INFRARED;
    range.field_of_view = field_of_view_;
    range.min_range = min_range_;
    range.max_range = max_range_;
    range.range = (buffer[8] + buffer[9] * 255) / 1000.0;
    range_pubs[buffer[3]]->publish(range);
  }
}
}  // namespace tof_sense

RCLCPP_COMPONENTS_REGISTER_NODE(tof_sense::TOFSenseNode)
