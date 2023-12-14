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
// Make P symmetric
auto make_symmetric(Eigen::Matrix<double, NUM_STATES, NUM_STATES> & P)
{
  return 0.5 * (P + P.transpose());
}

Eigen::Matrix<double, 1, NUM_STATES> TrajectoryLQR::get_feedback_gain(
  const rclcpp::Duration & duration_since_start)
{
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
  return K_vec_.at(idx);
}

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

  trajectory_active_lqr_ptr_ = std::make_shared<TrajectoryLQR>();

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
  parseGains();
  return true;
}

Eigen::Vector<double, NUM_STATES> get_state_from_point(
  trajectory_msgs::msg::JointTrajectoryPoint point)
{
  // TODO(anyone) create map to get the correct joint order
  // theta, theta_dot, x, x_dot
  if (point.positions.size() != 2)
  {
    throw std::runtime_error("JointTrajectoryPoint.positions does not have the correct size.");
  }
  if (point.velocities.size() != 2)
  {
    throw std::runtime_error("JointTrajectoryPoint.velocities does not have the correct size.");
  }
  return Eigen::Vector<double, NUM_STATES>{
    point.positions[1], point.velocities[1], point.positions[0], point.velocities[0]};
}

bool CartpoleLqrTrajectoryPlugin::computeControlLawNonRT_impl(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & trajectory)
{
  auto start_time = this->node_->now();

  // parameters
  auto N = Eigen::Matrix<double, 1, NUM_STATES>::Zero();

  Eigen::Matrix<double, 1, NUM_STATES> Ks;
  Eigen::Matrix<double, NUM_STATES, NUM_STATES> Ps;
  auto x_R = get_state_from_point(trajectory->points.back());
  Eigen::Vector<double, 1> u_R{0};
  calcLQR_steady(x_R, u_R, dt_, Q_, R_, N, Ks, Ps);

  auto new_trajectory_gains = std::make_shared<TrajectoryLQR>();

  if (trajectory->points.size() == 1)
  {
    // use steady-state gain
    new_trajectory_gains->K_vec_.push_back(Ks);
    new_trajectory_gains->time_vec_.push_back(rclcpp::Duration::from_seconds(dt_));
  }
  else
  {
    Eigen::Matrix<double, 1, NUM_STATES> K;
    Eigen::Matrix<double, NUM_STATES, NUM_STATES> Phi;
    Eigen::Matrix<double, NUM_STATES, 1> Gamma;
    Eigen::Matrix<double, NUM_STATES, NUM_STATES> C;
    C.setIdentity();  // we can measure every state
    Eigen::Matrix<double, NUM_STATES, NUM_STATES> P(Ps), P_new;
    Eigen::Vector<double, NUM_STATES> x_p1;
    Eigen::Vector<double, NUM_STATES> x;
    Eigen::Vector<double, 1> u{0};

    // reserve space for feedback gains for every point in trajectory
    // this is a design choice to save memory of the gains, theoretically we should
    // save the gains at the dt-grid.
    new_trajectory_gains->K_vec_.resize(trajectory->points.size() - 1);
    new_trajectory_gains->time_vec_.resize(trajectory->points.size() - 1);

    double t_traj = get_time_from_duration(trajectory->points.back().time_from_start);
    auto idx_traj = trajectory->points.size();

    // iterate Riccati equation, backwards in time
    while (t_traj > 0.0)
    {
      // TODO(christophfroehlich) interpolate from time or use trajectory class?
      const trajectory_msgs::msg::JointTrajectoryPoint point = trajectory->points.at(idx_traj - 1);
      x = get_state_from_point(point);
      // system input acceleration, derivative of velocity
      // we don't have acceleration in trajectory, so we have to calculate it
      if (idx_traj < trajectory->points.size())
      {
        double dt_traj = get_time_from_duration(trajectory->points.at(idx_traj).time_from_start) -
                         get_time_from_duration(point.time_from_start);
        x_p1 = get_state_from_point(trajectory->points.at(idx_traj));
        u[0] = (x_p1(3) - x(3)) / dt_traj;
      }
      get_linear_system_matrices(x, u, dt_, Phi, Gamma);

      K = -(R_ + Gamma.transpose() * P * Gamma).inverse() * (N + Gamma.transpose() * P * Phi);
      P_new = (Q_ + Phi.transpose() * P * Phi) + (N + Gamma.transpose() * P * Phi).transpose() * K;
      P = 0.5 * (P_new + P_new.transpose());

      // BEGIN: This part here is for exemplary purposes - Please do not copy to your production
      // code
      // DEBUG: wait to check if RT buffer works well
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      // END: This part here is for exemplary purposes - Please do not copy to your production code

      t_traj -= dt_;
      if (t_traj < get_time_from_duration(trajectory->points.at(idx_traj - 1).time_from_start))
      {
        idx_traj--;
        if (idx_traj == 0lu)
        {
          // we reached the start of the trajectory
          break;
        }
        // save feedback gain for this point
        new_trajectory_gains->K_vec_.at(idx_traj - 1) = K;
        new_trajectory_gains->time_vec_.at(idx_traj - 1) =
          trajectory->points.at(idx_traj).time_from_start;
      }
    }
  }

  trajectory_next_lqr_ptr_.writeFromNonRT(new_trajectory_gains);

  auto end_time = this->node_->now();
  auto duration = end_time - start_time;
  RCLCPP_INFO(
    node_->get_logger(),
    "[CartpoleLqrTrajectoryPlugin] computed control law for %lu points in %es.",
    trajectory->points.size(), duration.seconds());

  new_trajectory_gains->print();

  return true;
}

bool CartpoleLqrTrajectoryPlugin::computeControlLawRT_impl(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & trajectory)
{
  // parameters
  auto N = Eigen::Matrix<double, 1, NUM_STATES>::Zero();

  Eigen::Matrix<double, 1, NUM_STATES> Ks;
  Eigen::Matrix<double, NUM_STATES, NUM_STATES> Ps;
  auto x_R = get_state_from_point(trajectory->points.back());
  Eigen::Vector<double, 1> u_R{0};
  calcLQR_steady(x_R, u_R, dt_, Q_, R_, N, Ks, Ps);

  auto new_trajectory_gains = std::make_shared<TrajectoryLQR>();

  // use steady-state gain
  new_trajectory_gains->K_vec_.push_back(Ks);
  new_trajectory_gains->time_vec_.push_back(rclcpp::Duration::from_seconds(dt_));
  trajectory_next_lqr_ptr_.writeFromNonRT(new_trajectory_gains);

  new_trajectory_gains->print();

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
    P = make_symmetric(P_new);
  }
  Ks = K;
  Ps = P;

  // std::cout << "Ks: " << Ks << std::endl;
  // std::cout << "Ps: " << Ps << std::endl;
}

bool CartpoleLqrTrajectoryPlugin::updateGainsRT()
{
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    parseGains();
  }

  return true;
}

void CartpoleLqrTrajectoryPlugin::parseGains()
{
  Eigen::Vector<double, NUM_STATES> Q_diag{params_.gains.Q.data()};
  Q_ = Eigen::Matrix<double, NUM_STATES, NUM_STATES>{Q_diag.asDiagonal()};
  R_ = Eigen::Matrix<double, 1, 1>{params_.gains.R};
  // this is part of controller_interface definition, and not defined in GPL params_
  // if not set explicitly, it will be the cm_update rate
  dt_ = 1. / node_->get_parameter("update_rate").as_int();

  RCLCPP_INFO_STREAM(node_->get_logger(), "[CartpoleLqrTrajectoryPlugin] Q: " << std::endl << Q_);
  RCLCPP_INFO_STREAM(node_->get_logger(), "[CartpoleLqrTrajectoryPlugin] R: " << std::endl << R_);
  RCLCPP_INFO_STREAM(node_->get_logger(), "[CartpoleLqrTrajectoryPlugin] dt: " << std::endl << dt_);
}

void CartpoleLqrTrajectoryPlugin::computeCommands(
  std::vector<double> & tmp_command, const trajectory_msgs::msg::JointTrajectoryPoint /*current*/,
  const trajectory_msgs::msg::JointTrajectoryPoint error,
  const trajectory_msgs::msg::JointTrajectoryPoint desired,
  const rclcpp::Duration & duration_since_start, const rclcpp::Duration & period)
{
  if (!trajectory_active_lqr_ptr_->is_empty())
  {
    auto e = get_state_from_point(error);
    auto K = trajectory_active_lqr_ptr_->get_feedback_gain(duration_since_start);
    // integrate acceleration from state feedback to velocity. consider sign of e!
    u_ += period.seconds() * (K.dot(-e));
    // set system input as desired velocity + integrated LQR control output
    tmp_command.at(map_cmd_to_joints_.at(0)) = desired.velocities.at(map_cmd_to_joints_.at(0)) + u_;
  }
}

void CartpoleLqrTrajectoryPlugin::reset()
{
  trajectory_active_lqr_ptr_->reset();
  u_ = 0;
}

void CartpoleLqrTrajectoryPlugin::start()
{
  // switch storage to new gains
  trajectory_active_lqr_ptr_ = *trajectory_next_lqr_ptr_.readFromRT();
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
