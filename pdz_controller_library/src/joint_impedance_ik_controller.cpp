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

#include <pdz_controller_library/default_robot_behavior_utils.hpp>
#include <pdz_controller_library/joint_impedance_ik_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <chrono>

using namespace std::chrono_literals;


namespace pdz_controller_library {
// ---------------------------------------------------------------------------
// update_stiffness_and_references
// ---------------------------------------------------------------------------
void JointImpedanceIkController::update_stiffness_and_references(){
  nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  //target by filtering
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  // Convert the rotation matrix to Euler angles
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
}



// ---------------------------------------------------------------------------
// (damped) pseudoInverse
// ---------------------------------------------------------------------------
inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
  double lambda_ = damped ? 0.2 : 0.0;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);   
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
      S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}       





// ---------------------------------------------------------------------------
// reference_pose_callback
// ---------------------------------------------------------------------------
void JointImpedanceIkController::reference_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    // Handle the incoming pose message
    
    std::cout << "received reference pose as:" << std::endl;
    std::cout << "position: " << msg->position.x << ", " << msg->position.y << ", " << msg->position.z << std::endl;
    std::cout << "orientation: " << msg->orientation.w << ", " << msg->orientation.x << ", " << msg->orientation.y << ", " << msg->orientation.z << std::endl;
    
    position_d_target_ = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    orientation_d_target_ = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    orientation_d_target_.normalize();  
}



// ---------------------------------------------------------------------------
// command_interface_configuration
// ---------------------------------------------------------------------------
controller_interface::InterfaceConfiguration
JointImpedanceIkController::command_interface_configuration() const {

  RCLCPP_INFO(get_node()->get_logger(), "Starting command_interface_configuration...");

  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/effort");
  }

  RCLCPP_INFO(get_node()->get_logger(), "command_interface_configuration completed successfully.");

  return config;
}



// ---------------------------------------------------------------------------
// state_interface_configuration
// ---------------------------------------------------------------------------
controller_interface::InterfaceConfiguration
JointImpedanceIkController::state_interface_configuration() const {

  RCLCPP_INFO(get_node()->get_logger(), "Starting state_interface_configuration...");

  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/velocity");
  }

  RCLCPP_INFO(get_node()->get_logger(), "state_interface_configuration completed successfully.");

  return config;
}



// ---------------------------------------------------------------------------
// on_init
// ---------------------------------------------------------------------------
CallbackReturn JointImpedanceIkController::on_init() {
  RCLCPP_INFO(get_node()->get_logger(), "on_init completed successfully.");
  return CallbackReturn::SUCCESS;
}


// ---------------------------------------------------------------------------
// on_configure
// ---------------------------------------------------------------------------
CallbackReturn JointImpedanceIkController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(get_node()->get_logger(), "Starting on_configure...");
  
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_node()->get_logger(), "on_configure checkpoint #1");

  try{
    std::string robot_description;
    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
    parameters_client->wait_for_service();
    auto future = parameters_client->get_parameters({"robot_description"});
    auto result = future.get();
    if(!result.empty()){
      robot_description = result[0].value_to_string();
      RCLCPP_INFO(get_node()->get_logger(), "'robot_description' parameter retrieved successfully.");
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "on_configure checkpoint #4");

    // Parse the URDF using Pinocchio
    //The robot_description parameter contains the URDF as a string.
    //The buildModelFromXML function parses the URDF and initializes the Pinocchio model.
    pinocchio::urdf::buildModelFromXML(robot_description, model_);
    data_ = pinocchio::Data(model_);
    RCLCPP_INFO(get_node()->get_logger(), "Pinocchio model parsed successfully.");

    end_effector_frame_id_ = model_.getFrameId("fr3_hand");
  
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load Pinocchio model: %s", e.what());
    return CallbackReturn::ERROR;
  }


  RCLCPP_INFO(get_node()->get_logger(), "on_configure completed successfully.");
  return CallbackReturn::SUCCESS;
}


// ---------------------------------------------------------------------------
// on_activate
// ---------------------------------------------------------------------------
CallbackReturn JointImpedanceIkController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  RCLCPP_INFO(get_node()->get_logger(), "Starting on_activate...");
  initialization_flag_ = true;
  elapsed_time_ = 0.0;
  dq_filtered.setZero();

  RCLCPP_INFO(get_node()->get_logger(), "Initializing joint_positions_ and joint_velocities_ vectors...");

  joint_positions_desired_.resize(num_joints_);
  joint_positions_current_.resize(num_joints_);
  joint_velocities_current_.resize(num_joints_);
  joint_efforts_current_.resize(num_joints_);

  RCLCPP_INFO(get_node()->get_logger(), "Initializing q_ and dq_ vectors and setting to zero...");
  
  q_.resize(model_.nq);
  dq_.resize(model_.nv);

  q_.setZero();
  dq_.setZero();

  RCLCPP_INFO(get_node()->get_logger(), "q_ size: %zu", q_.size());
  RCLCPP_INFO(get_node()->get_logger(), "dq_ size: %zu", dq_.size());
  for (size_t i = 0; i < model_.names.size(); ++i) {
    std::cout << i << ": " << model_.names[i] << std::endl;
  }

  update_joint_states();

  joint_positions_desired_ = joint_positions_current_;

  RCLCPP_INFO(get_node()->get_logger(), "Initializing Jacobian and pseudoinverse matrices...");

  jacobian.resize(6, model_.nv);
  jacobian.setZero();
  jacobian_pinv.resize(model_.nv, 6);
  jacobian_pinv.setZero();
  jacobian_transpose_pinv.resize(model_.nv, 6);
  jacobian_transpose_pinv.setZero();

  RCLCPP_INFO(get_node()->get_logger(), "Jacobian size: %zu x %zu", jacobian.rows(), jacobian.cols());

  RCLCPP_INFO(get_node()->get_logger(), "Computing forward kinematics and frame placements...");

  pinocchio::forwardKinematics(model_, data_, q_);
  pinocchio::updateFramePlacements(model_, data_);

  Eigen::Affine3d initial_transform;
  initial_transform.linear() = data_.oMf[end_effector_frame_id_].rotation();  // Extract rotation
  initial_transform.translation() = data_.oMf[end_effector_frame_id_].translation();
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  position_d_target_ = position_d_;
  orientation_d_target_ = orientation_d_;


  joint_positions_desired_eigen.setZero();
  joint_positions_current_eigen.setZero();
  joint_positions_desired_eigen = joint_positions_current_eigen;
  joint_velocities_current_eigen.setZero();
  
  q_des_ik_.resize(model_.nq);
  q_des_ik_ = q_;

  // Create the subscriber in the on_activate method
  desired_pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::Pose>(
    "/joint_impedance_controller/reference_pose", 1, std::bind(&JointImpedanceIkController::reference_pose_callback, this, std::placeholders::_1));


  RCLCPP_INFO(get_node()->get_logger(), "on_activate completed successfully.");
  return CallbackReturn::SUCCESS;
}



// ---------------------------------------------------------------------------
// on_deactivate
// ---------------------------------------------------------------------------
controller_interface::CallbackReturn JointImpedanceIkController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // franka_cartesian_pose_->release_interfaces();
  RCLCPP_INFO(get_node()->get_logger(), "Controller deactivated.");
  return CallbackReturn::SUCCESS;
}



// ---------------------------------------------------------------------------
// update_joint_states
// ---------------------------------------------------------------------------
void JointImpedanceIkController::update_joint_states() {
  for (auto i = 0; i < num_joints_; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);
    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    joint_positions_current_[i] = position_interface.get_value();
    q_[i] = joint_positions_current_[i];

    joint_velocities_current_[i] = velocity_interface.get_value();
    dq_[i] = joint_velocities_current_[i]; // Update dq with current joint velocities
  }
}


// ---------------------------------------------------------------------------
// update (1kHz control loop)
// ---------------------------------------------------------------------------
controller_interface::return_type JointImpedanceIkController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

  if(counter_ == 0){
    RCLCPP_INFO(get_node()->get_logger(), "Starting control loop updates...");
  }

  update_joint_states();

  // --- Forward Kinematics ---
  pinocchio::forwardKinematics(model_, data_, q_);
  pinocchio::updateFramePlacements(model_, data_);

  // --- Update current EE pose ---
  Eigen::Affine3d transform;
  transform.linear() = data_.oMf[end_effector_frame_id_].rotation();  // Extract rotation
  transform.translation() = data_.oMf[end_effector_frame_id_].translation();  // Extract translation
  position_model = transform.translation();   // update the existing Vector3d
  orientation_model = Eigen::Quaterniond(transform.rotation());  // update the existing Quaterniond
  orientation_model.normalize(); // Ensure the quaternion is normalized
  orientation_d_.normalize(); // Ensure the desired orientation is normalized


  // --- error ---
  pinocchio::SE3 current_pose = data_.oMf[end_effector_frame_id_];
  pinocchio::SE3 desired_pose(orientation_d_.toRotationMatrix(), position_d_);
  pinocchio::SE3 pose_error = current_pose.actInv(desired_pose);
  Eigen::Matrix<double, 6, 1> error;
  error = pinocchio::log6(pose_error).toVector();
  
  error.head(3) *= kp_pos; // Scale position error
  error.tail(3) *= kp_ori; // Scale orientation error
  

  // --- Jacobian ---
  pinocchio::computeFrameJacobian(model_, data_, q_, end_effector_frame_id_, pinocchio::LOCAL, jacobian);
  
  Eigen::Matrix<double, 6, 7> jacobian_7 = jacobian.leftCols(7); // Extract the part of the Jacobian corresponding to the 7 arm joints

  // --- Inverse Differential Kinematics ---
  pseudoInverse(jacobian_7, jacobian_pinv, true);

  if (!jacobian_pinv.allFinite()) {
    RCLCPP_ERROR(rclcpp::get_logger("joint_impedance_ik_controller"), "Pseudo-inverse contains NaN!");
    jacobian_pinv.setZero(); // Set to zero or some safe value to prevent sending NaN torques to the robot
  }

  Eigen::VectorXd dq_ik = jacobian_pinv * error;

  
  Eigen::VectorXd dq_ik_full = Eigen::VectorXd::Zero(model_.nv); // Full velocity vector for all joints (including gripper joints) 
  dq_ik_full.head(7) = dq_ik;
    

  // --- Current joints ---
  joint_positions_current_eigen = Eigen::Map<Vector7d>(joint_positions_current_.data());
  joint_velocities_current_eigen = Eigen::Map<Vector7d>(joint_velocities_current_.data());

  // Integrate for desired joint positions
  q_des_ik_ = pinocchio::integrate(model_, q_des_ik_, alpha * dq_ik_full * dt); // Use Pinocchio's integrate function for better handling of joint limits and singularities
  joint_positions_desired_eigen = q_des_ik_.head(7);

  // tau_friction = tau_friction_external;
  auto tau_d_calculated = compute_torque_command(
  joint_positions_desired_eigen, joint_positions_current_eigen, joint_velocities_current_eigen);  
  
  if (!tau_d_calculated.allFinite()) {
    RCLCPP_ERROR(rclcpp::get_logger("joint_impedance_ik_controller"), "tau_d_calculated contains NaN!");
    tau_d_calculated.setZero(); // Set to zero or some safe value to prevent sending NaN torques to the robot
  }

  for (int i = 0; i < tau_d_calculated.size(); i++) {
    tau_d_calculated(i) = std::clamp(tau_d_calculated(i), -87.0, 87.0);  // Clamp to ±87 Nm for Franka FR3
  }

  //set torques
  for (int i = 0; i < num_joints_; i++) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }

  // std::cout << counter_ << std::endl;
  if (counter_ % 1000 == 0) {
    std::cout << "-------------------------------------------------------------------------------------" << std::endl;
    //std::cout << "friction torque: " << tau_friction.transpose() << std::endl;
    std::cout << "coriolis compensation: " << (pinocchio::computeCoriolisMatrix(model_, data_, q_, dq_) * dq_).head(7).transpose() << std::endl;
    std::cout << "gravity compensation: " << (pinocchio::computeGeneralizedGravity(model_, data_, q_)).head(7).transpose() << std::endl;
    std::cout << "commanded torque: " << tau_d_calculated.transpose() << std::endl;
    std::cout << "current joint configuration: " << joint_positions_current_eigen.transpose() << std::endl;
    std::cout << "current joint velocities: " << joint_velocities_current_eigen.transpose() << std::endl;
    std::cout << "desired joint configuration: " << joint_positions_desired_eigen.transpose() << std::endl;
    std::cout << "joint error: " << (joint_positions_desired_eigen - joint_positions_current_eigen).transpose() << std::endl;
    //std::cout << "joint error: " << (joint_positions_desired_eigen - joint_positions_current_eigen).transpose() << std::endl;
    //std::cout << "equivalent stiffness: " << (jacobian.transpose() * K_cartesian_desired * jacobian).diagonal().transpose() << std::endl;
    std::cout << "EE position: " << position_model.transpose() << std::endl;
    std::cout << "EE orientation: " << orientation_model.coeffs().transpose() << std::endl;
    std::cout << "desired EE position: " << position_d_target_.transpose() << std::endl;
    std::cout << "desired EE orientation: " << orientation_d_target_.coeffs().transpose() << std::endl;
    std::cout << "pose error: " << error.transpose() << std::endl;
    std::cout << "dq_ik: " << dq_ik.transpose() << std::endl;
    std::cout << "jacobian: " << std::endl << jacobian_7 << std::endl;
    std::cout << "jacobian pseudo-inverse: " << std::endl << jacobian_pinv << std::endl;
    std::cout << "-------------------------------------------------------------------------------------" << std::endl;
  }
  counter_++;
  update_stiffness_and_references();

  return controller_interface::return_type::OK;

}



// ---------------------------------------------------------------------------
// compute_torque_command
// ---------------------------------------------------------------------------
Vector7d JointImpedanceIkController::compute_torque_command(
    const Vector7d& joint_positions_desired,
    const Vector7d& joint_positions_current,
    const Vector7d& joint_velocities_current) {

  
  // --- coriolis ---
  Eigen::VectorXd coriolis_matrix = pinocchio::computeCoriolisMatrix(model_, data_, q_, dq_) * dq_;
  Vector7d coriolis = coriolis_matrix.head(7);
  

  // --- gravity ---
  Eigen::VectorXd gravity_vector = pinocchio::computeGeneralizedGravity(model_, data_, q_);
  Vector7d gravity = gravity_vector.head(7);

  
  // --- low-pass filter ---
  const double kAlpha = 0.01;
  dq_filtered = (1 - kAlpha) * dq_filtered + kAlpha * joint_velocities_current;

  
  // --- error ---
  Vector7d q_error = joint_positions_desired - joint_positions_current;

  
  // --- impedance control law ---
  Vector7d tau = k_gains_.cwiseProduct(q_error) - d_gains_.cwiseProduct(dq_filtered);


  // Add Coriolis and gravity compensation to the torque command 
  // (Note: remove gravity when using gazebo, as it already includes gravity compensation and will overcompensate otherwise)
  return tau + coriolis; // + gravity; 
}



// ---------------------------------------------------------------------------
// assign_parameters
// ---------------------------------------------------------------------------
bool JointImpedanceIkController::assign_parameters() {

  RCLCPP_INFO(get_node()->get_logger(), "arm_id parameter retrieved successfully: %s", arm_id_.c_str());

  RCLCPP_INFO(get_node()->get_logger(), "Retrieving and checking size of k_gains and d_gains...");

  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return false;
  }

  if (k_gains.size() != static_cast<uint>(num_joints_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints_, k_gains.size());
    return false;
  }

  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return false;
  }

  if (d_gains.size() != static_cast<uint>(num_joints_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints_, d_gains.size());
    return false;
  }

  RCLCPP_INFO(get_node()->get_logger(), "k_gains and d_gains retrieved successfully, assigning to member variables...");

  k_gains_.resize(num_joints_);
  d_gains_.resize(num_joints_);

  for (int i = 0; i < num_joints_; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }

  RCLCPP_INFO(get_node()->get_logger(), "Parameters assigned successfully.");

  return true;
}



}  // namespace pdz_controller_library
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(pdz_controller_library::JointImpedanceIkController,
                       controller_interface::ControllerInterface)


/*
Send reference pose example:

ros2 topic pub --once /joint_impedance_controller/reference_pose geometry_msgs/msg/Pose "position:
  x: 0.2
  y: 0.2
  z: 0.5
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0"

*/