// Copyright 2023 AIT Austrian Institute of Technology
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

#include "ros2_control_demo_example_xx/cartpole_lqr_trajectory_plugin.hpp"

namespace ros2_control_demo_example_xx
{

bool CartpoleLqrTrajectoryPlugin::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::vector<size_t> map_cmd_to_joints)
{
  node_ = node;
  map_cmd_to_joints_ = map_cmd_to_joints;

  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(node_);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return false;
  }

  return true;
}

bool CartpoleLqrTrajectoryPlugin::configure()
{
  try
  {
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during configure stage with message: %s \n", e.what());
    return false;
  }

  // parse read-only params
  num_cmd_joints_ = params_.command_joints.size();
  if (num_cmd_joints_ == 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "[CartpoleLqrTrajectoryPlugin] No command joints specified.");
    return false;
  }
  if (num_cmd_joints_ != map_cmd_to_joints_.size())
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[CartpoleLqrTrajectoryPlugin] map_cmd_to_joints has to be of size num_cmd_joints.");
    return false;
  }
  pids_.resize(num_cmd_joints_);
  ff_velocity_scale_.resize(num_cmd_joints_);

  return true;
};

bool CartpoleLqrTrajectoryPlugin::activate()
{
  params_ = param_listener_->get_params();
  updateGains();
  return true;
};

bool CartpoleLqrTrajectoryPlugin::updateGainsRT()
{
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    updateGains();
  }

  return true;
}

void CartpoleLqrTrajectoryPlugin::updateGains()
{
  for (size_t i = 0; i < num_cmd_joints_; ++i)
  {
    RCLCPP_DEBUG(
      node_->get_logger(), "[CartpoleLqrTrajectoryPlugin] params_.command_joints %lu : %s", i,
      params_.command_joints[i].c_str());

    const auto & gains = params_.gains.command_joints_map.at(params_.command_joints[i]);
    pids_[i] = std::make_shared<control_toolbox::Pid>(
      gains.p, gains.i, gains.d, gains.i_clamp, -gains.i_clamp);
    ff_velocity_scale_[i] = gains.ff_velocity_scale;

    RCLCPP_DEBUG(node_->get_logger(), "[CartpoleLqrTrajectoryPlugin] gains.p: %f", gains.p);
    RCLCPP_DEBUG(
      node_->get_logger(), "[CartpoleLqrTrajectoryPlugin] ff_velocity_scale_: %f",
      ff_velocity_scale_[i]);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[CartpoleLqrTrajectoryPlugin] Loaded PID gains from ROS parameters for %lu joint(s).",
    num_cmd_joints_);
}

void CartpoleLqrTrajectoryPlugin::computeCommands(
  std::vector<double> & tmp_command, const trajectory_msgs::msg::JointTrajectoryPoint /*current*/,
  const trajectory_msgs::msg::JointTrajectoryPoint error,
  const trajectory_msgs::msg::JointTrajectoryPoint desired,
  const rclcpp::Duration & /*duration_since_start*/, const rclcpp::Duration & period)
{
  // Update PIDs
  for (auto i = 0ul; i < num_cmd_joints_; ++i)
  {
    tmp_command[map_cmd_to_joints_[i]] =
      (desired.velocities[map_cmd_to_joints_[i]] * ff_velocity_scale_[i]) +
      pids_[i]->computeCommand(
        error.positions[map_cmd_to_joints_[i]], error.velocities[map_cmd_to_joints_[i]],
        (uint64_t)period.nanoseconds());
  }
}

void CartpoleLqrTrajectoryPlugin::reset()
{
  for (const auto & pid : pids_)
  {
    if (pid)
    {
      pid->reset();
    }
  }
}

}  // namespace ros2_control_demo_example_xx

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_xx::CartpoleLqrTrajectoryPlugin,
  joint_trajectory_controller_plugins::TrajectoryControllerBase)
