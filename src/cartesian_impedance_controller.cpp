// Copyright (c) 2021 Franka Emika GmbH
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

#include <pdz_controller_library/cartesian_impedance_controller.hpp>
#include <pdz_controller_library/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace {

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}

namespace pdz_controller_library {

// ---------------------------------------------------------------------------
// update_stiffness_and_references
// ---------------------------------------------------------------------------
void CartesianImpedanceController::update_stiffness_and_references(){
  //target by filtering
  /** at the moment we do not use dynamic reconfigure and control the robot via D, K and T **/
  //K = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * K;
  //D = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * D;
  nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  //std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
}



// ---------------------------------------------------------------------------
// arrayToMatrix for 7D vectors
// ---------------------------------------------------------------------------
void CartesianImpedanceController::arrayToMatrix(const std::array<double,7>& inputArray, Eigen::Matrix<double,7,1>& resultMatrix)
{
 for(long unsigned int i = 0; i < 7; ++i){
     resultMatrix(i,0) = inputArray[i];
   }
}



// ---------------------------------------------------------------------------
// arrayToMatrix for 6D vectors
// ---------------------------------------------------------------------------
void CartesianImpedanceController::arrayToMatrix(const std::array<double,6>& inputArray, Eigen::Matrix<double,6,1>& resultMatrix)
{
 for(long unsigned int i = 0; i < 6; ++i){
     resultMatrix(i,0) = inputArray[i];
   }
}



// ---------------------------------------------------------------------------
// saturateTorqueRate
// ---------------------------------------------------------------------------
Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
  const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
  const Eigen::Matrix<double, 7, 1>& tau_J_d_M) {  
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
  double difference = tau_d_calculated[i] - tau_J_d_M[i];
  tau_d_saturated[i] = tau_J_d_M[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}



// ---------------------------------------------------------------------------
// pseudoInverse
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
// command_interface_configuration
// ---------------------------------------------------------------------------
controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}



// ---------------------------------------------------------------------------
// state_interface_configuration
// ---------------------------------------------------------------------------
controller_interface::InterfaceConfiguration CartesianImpedanceController::state_interface_configuration()
  const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    state_interfaces_config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/position");
    state_interfaces_config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/velocity");
  }

  const std::string full_interface_name = robot_name_ + "/" + state_interface_name_;

  return state_interfaces_config;
}



// ---------------------------------------------------------------------------
// on_init
// ---------------------------------------------------------------------------
CallbackReturn CartesianImpedanceController::on_init() {
  UserInputServer input_server_obj(&position_d_target_, &rotation_d_target_, &K, &D, &T);
  std::thread input_thread(&UserInputServer::main, input_server_obj, 0, nullptr);
  input_thread.detach();
  RCLCPP_INFO(get_node()->get_logger(), "on_init completed successfully.");
  return CallbackReturn::SUCCESS;
}



// ---------------------------------------------------------------------------
// on_configure
// ---------------------------------------------------------------------------
CallbackReturn CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  try {

    RCLCPP_INFO(get_node()->get_logger(), "Starting on_configure...");

    // Retrieve the robot_description parameter
    // This retrieves the robot_description parameter from the ROS 2 parameter server.
    // If the parameter is not found, an error is logged, and the controller fails to configure.
    std::string robot_description;
    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
    parameters_client->wait_for_service();
    auto future = parameters_client->get_parameters({"robot_description"});
    auto result = future.get();
    if (!result.empty()) {
      robot_description = result[0].value_to_string();
      RCLCPP_INFO(get_node()->get_logger(), "'robot_description' parameter retrieved successfully.");
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
      return CallbackReturn::ERROR;
    }
    // Parse the URDF using Pinocchio
    //The robot_description parameter contains the URDF as a string.
    //The buildModelFromXML function parses the URDF and initializes the Pinocchio model.
    pinocchio::urdf::buildModelFromXML(robot_description, model_);
    data_ = pinocchio::Data(model_);
    RCLCPP_INFO(get_node()->get_logger(), "Pinocchio model parsed successfully.");
  
    //// Set the end-effector frame ID
    //// Replace "panda_hand" with the name of your robot's end-effector frame as defined in the URDF.
    //// This frame is used for Cartesian impedance control.
    end_effector_frame_id_ = model_.getFrameId("fr3_hand"); // Replace "panda_hand" with your actual frame name
    //RCLCPP_INFO(get_node()->get_logger(), "Pinocchio model loaded successfully.");
  } 
  catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load Pinocchio model: %s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "on_configure completed successfully.");
  return CallbackReturn::SUCCESS;
}



// ---------------------------------------------------------------------------
// on_activate
// ---------------------------------------------------------------------------
CallbackReturn CartesianImpedanceController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  // Reset initialization flag on activation
  first_valid_joint_state_received_ = false;
  RCLCPP_INFO(get_node()->get_logger(), "Controller activated. Waiting for first valid joint state.");
  

  std::cout << "Available frames in the model:" << std::endl;
  for (const auto& frame : model_.frames) {
  std::cout << frame.name << std::endl;
  }
  std::cout << "Number of available velocities:" << model_.nv << std::endl;
  dq_.resize(model_.nv);
  q_.resize(model_.nq);

  RCLCPP_INFO(get_node()->get_logger(), "model nq = %d, nv = %d", model_.nq, model_.nv);

  RCLCPP_INFO(get_node()->get_logger(), "q_ size: %zu", q_.size());
  RCLCPP_INFO(get_node()->get_logger(), "dq_ size: %zu", dq_.size());
  for (size_t i = 0; i < model_.names.size(); ++i) {
    std::cout << i << ": " << model_.names[i] << std::endl;
  }
  updateJointStates();
  jacobian.resize(6, model_.nv);
  jacobian_transpose_pinv.resize(model_.nv, 6);
  pinocchio::forwardKinematics(model_, data_, q_);
  pinocchio::updateFramePlacements(model_, data_);
  //Eigen::Affine3d initial_transform(data_.oMf[end_effector_frame_id_]);
  Eigen::Affine3d transform;
  transform.linear() = data_.oMf[end_effector_frame_id_].rotation();  // Extract rotation
  transform.translation() = data_.oMf[end_effector_frame_id_].translation();  // Extract translation
  position_d_ = transform.translation();
  orientation_d_ = Eigen::Quaterniond(transform.rotation());
  std::cout << "Completed Activation process" << std::endl;
  return CallbackReturn::SUCCESS;
}



// ---------------------------------------------------------------------------
// on_deactivate
// ---------------------------------------------------------------------------
controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  // franka_robot_model_->release_interfaces();
  RCLCPP_INFO(get_node()->get_logger(), "Controller deactivated.");
  return CallbackReturn::SUCCESS;
}



// ---------------------------------------------------------------------------
// convertToStdArray
// ---------------------------------------------------------------------------
std::array<double, 6> CartesianImpedanceController::convertToStdArray(const geometry_msgs::msg::WrenchStamped& wrench) {
    std::array<double, 6> result;
    result[0] = wrench.wrench.force.x;
    result[1] = wrench.wrench.force.y;
    result[2] = wrench.wrench.force.z;
    result[3] = wrench.wrench.torque.x;
    result[4] = wrench.wrench.torque.y;
    result[5] = wrench.wrench.torque.z;
    return result;
}



// ---------------------------------------------------------------------------
// updateJointStates
// ---------------------------------------------------------------------------
void CartesianImpedanceController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);
    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}



// ---------------------------------------------------------------------------
// update (1kHz control loop)
// ---------------------------------------------------------------------------
controller_interface::return_type CartesianImpedanceController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {  
  updateJointStates();
  
  // --- Delay Lambda computation until first valid joint state is received --
  if (!first_valid_joint_state_received_) {
    RCLCPP_INFO_ONCE(rclcpp::get_logger("cartesian_impedance_controller"), "Waiting for first valid joint state...");
    first_valid_joint_state_received_ = true;
    // Set all torques to zero during initialization
    for (size_t i = 0; i < 7; ++i) {
      command_interfaces_[i].set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }
  
  Eigen::VectorXd dynamic_torques = pinocchio::rnea(model_, data_, q_,  dq_, Eigen::VectorXd::Zero(model_.nv)); // 
  M = pinocchio::crba(model_, data_, q_); // rigid body algorithm
  pinocchio::computeFrameJacobian(model_, data_, q_, end_effector_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
  pinocchio::forwardKinematics(model_, data_, q_);
  pinocchio::updateFramePlacements(model_, data_);
  Eigen::MatrixXd g = pinocchio::computeGeneralizedGravity(model_, data_, q_);
  coriolis = dynamic_torques - g;
  //Eigen::Affine3d transform(data_.oMf[end_effector_frame_id_]);
  Eigen::Affine3d transform;
  transform.linear() = data_.oMf[end_effector_frame_id_].rotation();  // Extract rotation
  transform.translation() = data_.oMf[end_effector_frame_id_].translation();  // Extract translation
  Eigen::Vector3d position = transform.translation();
  Eigen::Quaterniond orientation(transform.rotation());
  orientation_d_target_ = Eigen::AngleAxisd(rotation_d_target_[0], Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(rotation_d_target_[1], Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(rotation_d_target_[2], Eigen::Vector3d::UnitZ());
  
  orientation.normalize();
  orientation_d_target_.normalize();

  error.head(3) << position - position_d_;

  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail(3) << -transform.rotation() * error.tail(3);

  double damping = 1e-6;
  Lambda = (jacobian * M.inverse() * jacobian.transpose() + damping * IDENTITY).inverse();

  if (!Lambda.allFinite()) {
    RCLCPP_ERROR(rclcpp::get_logger("cartesian_impedance_controller"), "Lambda contains NaN!");
  }

    // correcting D to be critically damped
  D =  D_gain* K.cwiseMax(0.0).cwiseSqrt() * Lambda.cwiseMax(0.0).diagonal().cwiseSqrt().asDiagonal();

  F_impedance = -1 * ((D * jacobian * dq_) + K * error);

  Eigen::VectorXd tau_nullspace(7), tau_d(7), tau_impedance(7);
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  //tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
  //                  jacobian.transpose() * jacobian_transpose_pinv) *
  //                  (nullspace_stiffness_ * config_control * (q_d_nullspace_ - q_) - //if config_control = true we control the whole robot configuration
  //                  (2.0 * sqrt(nullspace_stiffness_)) * dq_);  // if config control ) false we don't care about the joint position

  tau_impedance = jacobian.topLeftCorner(6,7).transpose() * Sm * F_impedance; //+ jacobian.transpose() * Sf * F_cmd;
  tau_d = tau_impedance + tau_nullspace + coriolis.head(7); //add nullspace and coriolis components to desired torque
  tau_d << saturateTorqueRate(tau_d, tau_J_d_M);  // Saturate torque rate to avoid discontinuities
  tau_J_d_M = tau_d;

  if (!tau_d.allFinite()) {
    RCLCPP_ERROR(rclcpp::get_logger("cartesian_impedance_controller"), "tau_d contains NaN!");
    tau_d.setZero(); // Set to zero or some safe value to prevent sending NaN torques to the robot
  }

  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }
  
  if (outcounter % 1000 == 0){
    std::cout << "-------------------------------------------------------------------------------------" << std::endl;
    // std::cout << "F_ext_robot [N]" << std::endl;
    std::cout << "dynamic torques" << dynamic_torques.transpose() << std::endl;
    std::cout << "g " << g.transpose() << std::endl;
    //std::cout << "Lambda: " << Lambda << std::endl;
    std::cout << "tau_d: " << tau_d.transpose() << std::endl;
    // std::cout << "--------" << std::endl;
    //std::cout << "tau_nullspace: " << tau_nullspace.transpose() << std::endl;
    // std::cout "tau_d: " << << "--------" << std::endl;
    //std::cout << "tau_impedance: " << tau_impedance.transpose() << std::endl;
    // std::cout << "--------" << std::endl;
    std::cout << "coriolis: " << coriolis.transpose() << std::endl;
    // std::cout << "Inertia scaling [m]: " << std::endl;
    // std::cout << T << std::endl;
    std::cout << "position: " << position.transpose() << std::endl;
    std::cout << "orientation: " << orientation << std::endl;
    std::cout << "-------------------------------------------------------------------------------------" << std::endl;
  }
  outcounter++;
  update_stiffness_and_references();
  return controller_interface::return_type::OK;
}
}

// namespace pdz_controller_library
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(pdz_controller_library::CartesianImpedanceController,
                       controller_interface::ControllerInterface)