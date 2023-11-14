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

#include <algorithm>

#include "ros2_control_demo_example_xx/cartpole_lqr_trajectory_plugin.hpp"

namespace ros2_control_demo_example_xx
{

bool CartpoleLqrTrajectoryPlugin::initialize(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::vector<size_t> map_cmd_to_joints)
{
  node_ = node;
  map_cmd_to_joints_ = map_cmd_to_joints;
  if (1 != map_cmd_to_joints_.size())
  {
    RCLCPP_ERROR(
      node_->get_logger(), "[CartpoleLqrTrajectoryPlugin] map_cmd_to_joints has to have size 1.");
    return false;
  }
  for (const auto & map_cmd_to_joint : map_cmd_to_joints_)
  {
    RCLCPP_INFO(
      node_->get_logger(), "[CartpoleLqrTrajectoryPlugin] map_cmd_to_joint: %lu", map_cmd_to_joint);
  }

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

  reset();

  return true;
};

bool CartpoleLqrTrajectoryPlugin::activate()
{
  params_ = param_listener_->get_params();
  updateGains();
  return true;
};

bool CartpoleLqrTrajectoryPlugin::computeControlLawNonRT_impl(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & trajectory)
{
  // parameters
  auto Q = Eigen::Matrix<double, 2, 2>::Identity();
  auto R = Eigen::Matrix<double, 1, 1>::Identity();
  auto N = Eigen::Matrix<double, 1, 2>::Zero();

  Eigen::Matrix<double, 1, 2> Ks;
  Eigen::Matrix<double, 2, 2> Ps;

  calcLQR_steady(Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Q, R, N, Ks, Ps);

  Eigen::Matrix<double, 1, 2> K;
  Eigen::Matrix<double, 2, 2> Phi;
  Eigen::Matrix<double, 2, 1> Gamma;
  Eigen::Matrix<double, 2, 2> C;
  C.setIdentity();
  Eigen::Matrix<double, 2, 2> P(Ps), P_new;
  // TODO(christophfroehlich) use a buffer to switch from RT thread to new gain
  K_vec_.clear();
  time_vec_.clear();

  // iterate Riccati equation
  for (const auto & point : trajectory->points)
  {
    // TODO(christophfroehlich) get x_DDot
    // TODO(christophfroehlich) create map to get the correct joint position
    get_linear_system_matrices(point.positions[1], 0.0, dt, Phi, Gamma);
    K = -(R + Gamma.transpose() * P * Gamma).inverse() * (N + Gamma.transpose() * P * Phi);
    P_new = (Q + Phi.transpose() * P * Phi) + (N + Gamma.transpose() * P * Phi).transpose() * K;
    P = 0.5 * (P_new + P_new.transpose());
    K_vec_.push_back(K);
    time_vec_.push_back(point.time_from_start);
  }
  // could be more efficient using reverse iterator
  std::reverse(K_vec_.begin(), K_vec_.end());

  for (const auto & mat : K_vec_)
  {
    std::cout << "K: " << mat << std::endl;
  }

  return true;
}

void CartpoleLqrTrajectoryPlugin::calcLQR_steady(
  Eigen::Vector2d q, Eigen::Vector2d qDot, Eigen::Matrix<double, 2, 2> Q,
  Eigen::Matrix<double, 1, 1> R, Eigen::Matrix<double, 1, 2> N, Eigen::Matrix<double, 1, 2> & Ks,
  Eigen::Matrix<double, 2, 2> & Ps)
{
  Eigen::Matrix<double, 2, 2> Phi;
  Eigen::Matrix<double, 2, 1> Gamma;
  Eigen::Matrix<double, 2, 2> C;
  C.setIdentity();

  double theta = q[1];

  get_linear_system_matrices(theta, 0.0, dt, Phi, Gamma);

  // solve steady-state Riccati equation iteratively
  Eigen::Matrix<double, 2, 2> P, P_new;
  Eigen::Matrix<double, 1, 2> K;
  P.setIdentity();

  P = P * 1e5;  // initial value of P
  int ct = 0;
  for (ct = 1; ct < 1e4; ct++)
  {
    K = -(R + Gamma.transpose() * P * Gamma).inverse() * (N + Gamma.transpose() * P * Phi);
    P_new = (Q + Phi.transpose() * P * Phi) + (N + Gamma.transpose() * P * Phi).transpose() * K;
    if ((P_new - P).norm() < 1e-3)
    {  // abort criterium
      break;
    }
    P = 0.5 * (P_new + P_new.transpose());
  }
  Ks = K;
  Ps = P;
}

bool CartpoleLqrTrajectoryPlugin::updateGainsRT()
{
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    updateGains();
  }

  return true;
}

void CartpoleLqrTrajectoryPlugin::updateGains() {}

void CartpoleLqrTrajectoryPlugin::computeCommands(
  std::vector<double> & tmp_command, const trajectory_msgs::msg::JointTrajectoryPoint /*current*/,
  const trajectory_msgs::msg::JointTrajectoryPoint error,
  const trajectory_msgs::msg::JointTrajectoryPoint desired,
  const rclcpp::Duration & duration_since_start, const rclcpp::Duration & period)
{
  if (K_vec_.size() > 0)
  {
    // TODO(christophfroehlich) create map to get the correct joint from error
    Eigen::Vector2d e{error.positions[1], error.velocities[1]};
    // TODO(christophfroehlich) interpolate K from time
    auto it = std::upper_bound(time_vec_.begin(), time_vec_.end(), duration_since_start);
    int idx;
    if (it == time_vec_.end())
    {
      idx = time_vec_.size() - 1;
    }
    else
    {
      idx = std::distance(time_vec_.begin(), it);
    }
    // integrate acceleration from state feedback to velocity
    u += period.seconds() * (K_vec_.at(idx).dot(e));
    // set system input as desired velocity + integrated LQR control output
    tmp_command.at(map_cmd_to_joints_.at(0)) = desired.velocities.at(map_cmd_to_joints_.at(0)) + u;
  }
}

void CartpoleLqrTrajectoryPlugin::reset()
{
  K_vec_.clear();
  u = 0;
}

void CartpoleLqrTrajectoryPlugin::get_linear_system_matrices(
  const double theta, const double x_ddot, const double dt, Eigen::Matrix<double, 2, 2> & Phi,
  Eigen::Matrix<double, 2, 1> & Gamma)
{
  // system parameters
  double lS = params_.system.lS;
  double mS = params_.system.mS;
  double g = params_.system.g;
  double IzzS = params_.system.IzzS;

  // Jacobians of continuous-time dynamics
  Eigen::Matrix<double, 2, 2> J_x;
  J_x(0, 0) = 0;
  J_x(0, 1) = 1;
  J_x(1, 0) = -0.2e1 * lS * mS * (g * cos(theta) - sin(theta) * x_ddot) / (lS * lS * mS + 4 * IzzS);
  J_x(1, 1) = 0;

  Eigen::Matrix<double, 2, 1> J_u(2);
  J_u(0, 0) = 0;
  J_u(1, 0) = -0.2e1 * lS * mS * cos(theta) / (lS * lS * mS + 4 * IzzS);

  // discrete-time linearized system matrices
  Phi = Eigen::Matrix<double, 2, 2>::Identity() + dt * J_x;
  Gamma = dt * J_u;
}

}  // namespace ros2_control_demo_example_xx

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_xx::CartpoleLqrTrajectoryPlugin,
  joint_trajectory_controller_plugins::TrajectoryControllerBase)
