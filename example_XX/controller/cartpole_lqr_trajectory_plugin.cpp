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
}

bool CartpoleLqrTrajectoryPlugin::activate()
{
  params_ = param_listener_->get_params();
  updateGains();
  return true;
}

Eigen::Vector<double, NUM_STATES> get_state_from_point(
  trajectory_msgs::msg::JointTrajectoryPoint point)
{
  // TODO(anyone) create map to get the correct joint order
  // theta, theta_dot, x, x_dot
  return Eigen::Vector<double, NUM_STATES>{
    point.positions[1], point.velocities[1], point.positions[0], point.velocities[0]};
}

bool CartpoleLqrTrajectoryPlugin::computeControlLawNonRT_impl(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & trajectory)
{
  // parameters
  auto N = Eigen::Matrix<double, 1, NUM_STATES>::Zero();

  Eigen::Matrix<double, 1, NUM_STATES> Ks;
  Eigen::Matrix<double, NUM_STATES, NUM_STATES> Ps;
  auto x_R = get_state_from_point(trajectory->points.back());
  Eigen::Vector<double, 1> u_R{0};
  calcLQR_steady(x_R, u_R, 0.1, Q_, R_, N, Ks, Ps);
  // std::cout << "Ks: " << Ks << std::endl;
  // std::cout << "Ps: " << Ps << std::endl;

  if (trajectory->points.size() == 1)
  {
    // use steady-state gain
    K_vec_.push_back(Ks);
    time_vec_.push_back(rclcpp::Duration::from_seconds(0.0));
  }
  else
  {
    Eigen::Matrix<double, 1, NUM_STATES> K;
    Eigen::Matrix<double, NUM_STATES, NUM_STATES> Phi;
    Eigen::Matrix<double, NUM_STATES, 1> Gamma;
    Eigen::Matrix<double, NUM_STATES, NUM_STATES> C;
    C.setIdentity();
    Eigen::Matrix<double, NUM_STATES, NUM_STATES> P(Ps), P_new;
    // TODO(christophfroehlich) use a buffer to switch from RT thread to new gain
    K_vec_.clear();
    time_vec_.clear();
    Eigen::Vector<double, NUM_STATES> x_p1;
    Eigen::Vector<double, NUM_STATES> x;
    Eigen::Vector<double, 1> u{0};
    auto get_time = [](const trajectory_msgs::msg::JointTrajectoryPoint & point)
    { return point.time_from_start.sec + point.time_from_start.nanosec * 1e-9; };
    double dt_traj;

    // iterate Riccati equation, backwards in time
    for (size_t i = trajectory->points.size(); i > 0; i--)
    {
      const trajectory_msgs::msg::JointTrajectoryPoint point = trajectory->points.at(i - 1);
      x = get_state_from_point(point);
      // system input acceleration, derivative of velocity
      // we don't have acceleration in trajectory, so we have to calculate it
      if (i == trajectory->points.size())
      {
        dt_traj = 0.1;
        u[0] = 0.0;
        x_p1 = x;
      }
      else
      {
        dt_traj = get_time(trajectory->points.at(i)) - get_time(point);
        x_p1 = get_state_from_point(trajectory->points.at(i));
        u[0] = (x_p1(3) - x(3)) / dt_traj;
      }
      get_linear_system_matrices(x, u, dt_traj, Phi, Gamma);

      K = -(R_ + Gamma.transpose() * P * Gamma).inverse() * (N + Gamma.transpose() * P * Phi);
      P_new = (Q_ + Phi.transpose() * P * Phi) + (N + Gamma.transpose() * P * Phi).transpose() * K;
      P = 0.5 * (P_new + P_new.transpose());

      K_vec_.push_back(K);
      time_vec_.push_back(point.time_from_start);
    }
    // could be more efficient using reverse iterator in computeCommand
    std::reverse(K_vec_.begin(), K_vec_.end());
  }

  // for (const auto & mat : K_vec_) {
  //   std::cout << "K: " << std::endl << mat << std::endl;
  // }

  return true;
}

void CartpoleLqrTrajectoryPlugin::calcLQR_steady(
  Eigen::Vector<double, NUM_STATES> x, Eigen::Vector<double, 1> u, double dt,
  Eigen::Matrix<double, NUM_STATES, NUM_STATES> Q, Eigen::Matrix<double, 1, 1> R,
  Eigen::Matrix<double, 1, NUM_STATES> N, Eigen::Matrix<double, 1, NUM_STATES> & Ks,
  Eigen::Matrix<double, NUM_STATES, NUM_STATES> & Ps)
{
  Eigen::Matrix<double, NUM_STATES, NUM_STATES> Phi;
  Eigen::Matrix<double, NUM_STATES, 1> Gamma;
  Eigen::Matrix<double, NUM_STATES, NUM_STATES> C;
  C.setIdentity();

  get_linear_system_matrices(x, u, dt, Phi, Gamma);

  // solve steady-state Riccati equation iteratively
  Eigen::Matrix<double, NUM_STATES, NUM_STATES> P, P_new;
  Eigen::Matrix<double, 1, NUM_STATES> K;
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

void CartpoleLqrTrajectoryPlugin::updateGains()
{
  Eigen::Vector<double, NUM_STATES> Q_diag{params_.gains.Q.data()};
  Q_ = Eigen::Matrix<double, NUM_STATES, NUM_STATES>{Q_diag.asDiagonal()};
  R_ = Eigen::Matrix<double, 1, 1>{params_.gains.R};
  dt_ = params_.dt;
  std::cout << "Q: " << std::endl << Q_ << std::endl;
  std::cout << "R: " << std::endl << R_ << std::endl;
  std::cout << "dt: " << std::endl << dt_ << std::endl;
}

void CartpoleLqrTrajectoryPlugin::computeCommands(
  std::vector<double> & tmp_command, const trajectory_msgs::msg::JointTrajectoryPoint /*current*/,
  const trajectory_msgs::msg::JointTrajectoryPoint error,
  const trajectory_msgs::msg::JointTrajectoryPoint desired,
  const rclcpp::Duration & duration_since_start, const rclcpp::Duration & period)
{
  if (K_vec_.size() > 0)
  {
    auto e = get_state_from_point(error);
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
    // integrate acceleration from state feedback to velocity. consider sign of e!
    u += period.seconds() * (K_vec_.at(idx).dot(-e));
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
  Eigen::Vector<double, NUM_STATES> x, Eigen::Vector<double, 1> u, const double dt,
  Eigen::Matrix<double, NUM_STATES, NUM_STATES> & Phi, Eigen::Matrix<double, NUM_STATES, 1> & Gamma)
{
  // states and input
  double theta = x[0];
  double x_ddot = u[0];

  // system parameters
  double lS = params_.system.lS;
  double mS = params_.system.mS;
  double g = params_.system.g;
  double IzzS = params_.system.IzzS;

  // Jacobians of continuous-time dynamics
  Eigen::Matrix<double, NUM_STATES, NUM_STATES> J_x =
    Eigen::Matrix<double, NUM_STATES, NUM_STATES>::Zero();
  J_x(0, 1) = 1;
  J_x(1, 0) = -0.2e1 * lS * mS * (g * cos(theta) - sin(theta) * x_ddot) / (lS * lS * mS + 4 * IzzS);
  J_x(2, 3) = 1;

  Eigen::Matrix<double, NUM_STATES, 1> J_u = Eigen::Matrix<double, NUM_STATES, 1>::Zero();
  J_u(1, 0) = -0.2e1 * lS * mS * cos(theta) / (lS * lS * mS + 4 * IzzS);
  J_u(3, 0) = 1;

  // discrete-time linearized system matrices
  Phi = Eigen::Matrix<double, NUM_STATES, NUM_STATES>::Identity() + dt * J_x;
  Gamma = dt * J_u;
}

}  // namespace ros2_control_demo_example_xx

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_xx::CartpoleLqrTrajectoryPlugin,
  joint_trajectory_controller_plugins::TrajectoryControllerBase)
