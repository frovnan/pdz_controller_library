// Copyright (c) 2023 Franka Robotics GmbH
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

#include <pdz_controller_library/joint_impedance_controller.hpp>
#include <pdz_controller_library/default_robot_behavior_utils.hpp>
#include <pdz_controller_library/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace pdz_controller_library {


// ---------------------------------------------------------------------------
// reference_configuration_callback
// ---------------------------------------------------------------------------
void JointImpedanceController::reference_configuration_callback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
{
    if (msg->positions.size() != static_cast<size_t>(num_joints)) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Received reference configuration with %zu positions, expected %d. Ignoring message.",
                  msg->positions.size(), num_joints);
      return;
    }

    RCLCPP_INFO(get_node()->get_logger(), "Received reference joint configuration.");
    for (int i = 0; i < num_joints; ++i) {
      q_goal(i) = msg->positions[i];
    }
}



// ---------------------------------------------------------------------------
// command_interface_configuration
// ---------------------------------------------------------------------------
controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const {

  RCLCPP_INFO(get_node()->get_logger(), "Starting command_interface_configuration...");

  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }

  RCLCPP_INFO(get_node()->get_logger(), "command_interface_configuration completed successfully.");

  return config;
}



// ---------------------------------------------------------------------------
// state_interface_configuration
// ---------------------------------------------------------------------------
controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const {

  RCLCPP_INFO(get_node()->get_logger(), "Starting state_interface_configuration...");

  std::cout << "STATE INTERFACE CONFIGURATION"  << std::endl;
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }

  RCLCPP_INFO(get_node()->get_logger(), "state_interface_configuration completed successfully.");

  return config;
}



// ---------------------------------------------------------------------------
// on_init
// ---------------------------------------------------------------------------
CallbackReturn JointImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "on_init completed successfully.");

  return CallbackReturn::SUCCESS;
}



// ---------------------------------------------------------------------------
// on_configure
// ---------------------------------------------------------------------------
CallbackReturn JointImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  RCLCPP_INFO(get_node()->get_logger(), "Starting on_configure...");
  
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints, k_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints, d_gains.size());
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  dq_filtered_.setZero();

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  pinocchio::urdf::buildModelFromXML(robot_description_, model_);
  data_ = pinocchio::Data(model_);
  RCLCPP_INFO(get_node()->get_logger(), "Pinocchio model parsed successfully.");

  RCLCPP_INFO(get_node()->get_logger(), "on_configure completed successfully.");

  return CallbackReturn::SUCCESS;
}



// ---------------------------------------------------------------------------
// on_activate
// ---------------------------------------------------------------------------
CallbackReturn JointImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  RCLCPP_INFO(get_node()->get_logger(), "Starting on_activate...");

  q_full.resize(model_.nq);
  q_full.setZero();
  dq_full.resize(model_.nv);
  dq_full.setZero();

  updateJointStates();
  
  dq_filtered_.setZero();

  q_ = q_full.head(7);  
  dq_ = dq_full.head(7);

  initial_q_ = q_;
  q_filtered_ = q_;
  q_goal = q_;

  elapsed_time_ = 0.0;

  reference_configuration_sub_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
      "/joint_impedance_controller/reference_configuration", 1,
      std::bind(&JointImpedanceController::reference_configuration_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "on_activate completed successfully.");

  return CallbackReturn::SUCCESS;
}



// ---------------------------------------------------------------------------
// update_joint_states
// ---------------------------------------------------------------------------
void JointImpedanceController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_full(i) = position_interface.get_value();
    dq_full(i) = velocity_interface.get_value();
  }
}


// ---------------------------------------------------------------------------
// update (1kHz control loop)
// ---------------------------------------------------------------------------
controller_interface::return_type JointImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

  if(counter_ == 0){
    RCLCPP_INFO(get_node()->get_logger(), "Starting control loop updates...");
  }

  updateJointStates();

  q_ = q_full.head(7);
  dq_ = dq_full.head(7);

  const double alpha = 0.01; // Smoothing factor for the low-pass filter
  q_filtered_ = (1 - alpha) * q_filtered_ + alpha * q_goal;
  dq_filtered_ = (1 - alpha) * dq_filtered_ + alpha * dq_;

  // --- impedance control law ---
  Vector7d tau_d_calculated = k_gains_.cwiseProduct(q_filtered_ - q_) - d_gains_.cwiseProduct(dq_filtered_);
  
  // --- coriolis ---
  Eigen::VectorXd coriolis_matrix = pinocchio::computeCoriolisMatrix(model_, data_, q_full, dq_full) * dq_full;
  Vector7d coriolis = coriolis_matrix.head(7);
  tau_d_calculated += coriolis;

  // --- gravity ---
  Eigen::VectorXd gravity_vector = pinocchio::computeGeneralizedGravity(model_, data_, q_full);
  Vector7d gravity = gravity_vector.head(7);
  // tau_d_calculated += gravity; // Note: remove gravity when using gazebo, as it already includes gravity compensation and will overcompensate otherwise

  for (int i = 0; i < tau_d_calculated.size(); i++) {
    tau_d_calculated(i) = std::clamp(tau_d_calculated(i), -87.0, 87.0);  // Clamp to ±87 Nm for Franka FR3
  }

  // set torques
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }

  if (counter_ % 1000 == 0) {
    std::cout << "-------------------------------------------------------------------------------------" << std::endl;
    std::cout << "coriolis compensation: " << coriolis.transpose() << std::endl;
    std::cout << "gravity compensation: " << gravity.transpose() << std::endl;
    std::cout << "commanded torque: " << tau_d_calculated.transpose() << std::endl;
    std::cout << "current joint configuration: " << q_.transpose() << std::endl;
    std::cout << "current joint velocities: " << dq_.transpose() << std::endl;
    std::cout << "desired joint configuration: " << q_goal.transpose() << std::endl;
    std::cout << "joint error: " << (q_goal - q_).transpose() << std::endl;
    std::cout << "-------------------------------------------------------------------------------------" << std::endl;
  }
  counter_++;

  return controller_interface::return_type::OK;
}



}  // namespace pdz_controller_library
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(pdz_controller_library::JointImpedanceController,
                       controller_interface::ControllerInterface)

                       
/*

Send reference joint configuration example:

ros2 topic pub --once /joint_impedance_controller/reference_configuration trajectory_msgs/msg/JointTrajectoryPoint "
positions:
- 0.0
- -0.3
- 0.0
- -1.5
- 0.0
- 1.2
- 0.7
"

*/
