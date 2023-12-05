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

#ifndef ROS2_CONTROL_DEMO_EXAMPLE_XX__CARTPOLE_LQR_TRAJECTORY_PLUGIN_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_XX__CARTPOLE_LQR_TRAJECTORY_PLUGIN_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "cartpole_lqr_trajectory_plugin_parameters.hpp"  // auto-generated by generate_parameter_library
#include "joint_trajectory_controller_plugins/trajectory_controller_base.hpp"

// shortcut
#define NUM_STATES 4
namespace ros2_control_demo_example_xx
{

double get_time(const builtin_interfaces::msg::Duration & time_from_start)
{
  return time_from_start.sec + time_from_start.nanosec * 1e-9;
};

class TrajectoryLQR
{
public:
  std::vector<Eigen::Matrix<double, 1, NUM_STATES>> K_vec_;
  std::vector<builtin_interfaces::msg::Duration> time_vec_;

  /**
   * @brief sample the feedback gain at the given time
   */
  Eigen::Matrix<double, 1, NUM_STATES> get_feedback_gain(
    const rclcpp::Duration & duration_since_start);

  /**
   * @brief resets internal storage
   */
  void reset()
  {
    K_vec_.clear();
    time_vec_.clear();
  };

  /**
   * @brief returns true if the trajectory is empty
   */
  bool is_empty() { return K_vec_.empty(); }

  /**
   * @brief print the gains on std::cout
   */
  void print()
  {
    for (size_t i = 0; i < K_vec_.size(); ++i)
    {
      std::cout << "at t: " << get_time(time_vec_.at(i)) << "s K: " << std::endl
                << K_vec_.at(i) << std::endl;
    }
  }
};

class CartpoleLqrTrajectoryPlugin
: public joint_trajectory_controller_plugins::TrajectoryControllerBase
{
public:
  bool initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::vector<size_t> map_cmd_to_joints) override;

  bool configure() override;

  bool activate() override;

  // computes the control law for a given trajectory, for non-RT thread
  bool computeControlLawNonRT_impl(
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & trajectory) override;

  // fast version of computeControlLawNonRT_impl for a single point only, for RT thread
  bool computeControlLawRT_impl(
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & trajectory) override;

  bool updateGainsRT() override;

  void computeCommands(
    std::vector<double> & tmp_command, const trajectory_msgs::msg::JointTrajectoryPoint current,
    const trajectory_msgs::msg::JointTrajectoryPoint error,
    const trajectory_msgs::msg::JointTrajectoryPoint desired,
    const rclcpp::Duration & duration_since_start, const rclcpp::Duration & period) override;

  void reset() override;

  void start() override;

protected:
  /**
   * @brief parse gains from parameter struct
   */
  void parseGains();

  void get_linear_system_matrices(
    Eigen::Vector<double, NUM_STATES> x, Eigen::Vector<double, 1> u, const double dt,
    Eigen::Matrix<double, NUM_STATES, NUM_STATES> & Phi,
    Eigen::Matrix<double, NUM_STATES, 1> & Gamma);
  void calcLQR_steady(
    Eigen::Vector<double, NUM_STATES> q, Eigen::Vector<double, 1> u, double dt,
    Eigen::Matrix<double, NUM_STATES, NUM_STATES> Q, Eigen::Matrix<double, 1, 1> R,
    Eigen::Matrix<double, 1, NUM_STATES> N, Eigen::Matrix<double, 1, NUM_STATES> & Ks,
    Eigen::Matrix<double, NUM_STATES, NUM_STATES> & Ps);
  double u_;

  // number of command joints
  size_t num_cmd_joints_;
  // map from joints in the message to command joints
  std::vector<size_t> map_cmd_to_joints_;

  // Parameters from ROS for ros2_control_demo_example_xx
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  // sampling time of the controller
  double dt_ = 0.01;
  // LQR cost function parameter for states
  Eigen::Matrix<double, NUM_STATES, NUM_STATES> Q_;
  // LQR cost function parameter for input
  Eigen::Matrix<double, 1, 1> R_;

  // storage for LQR gains
  std::shared_ptr<TrajectoryLQR> trajectory_active_lqr_ptr_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<TrajectoryLQR>> trajectory_next_lqr_ptr_;
};

}  // namespace ros2_control_demo_example_xx

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_XX__CARTPOLE_LQR_TRAJECTORY_PLUGIN_HPP_
