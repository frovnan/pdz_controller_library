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

#include <pdz_controller_library/admittance_controller.hpp>
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
// AdmittanceController
// ---------------------------------------------------------------------------
AdmittanceController::AdmittanceController(){
  control_mode = POSITION_CONTROL; // sets control mode
  //input_control_mode = TARGET_POSITION; // sets position control mode
  //D =  2 * K.cwiseSqrt(); // set critical damping from the get go
  Kd = 2 * Kp.cwiseSqrt();
}



// ---------------------------------------------------------------------------
// update_stiffness_and_references
// ---------------------------------------------------------------------------
void AdmittanceController::update_stiffness_and_references(){
  //target by filtering
  /** at the moment we do not use dynamic reconfigure and control the robot via D, K and T **/
  //K = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * K;
  //D = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * D;
  nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  //std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  
  // Convert the rotation matrix to Euler angles
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

  /* std::cout << "position_d update_stiffness_and_references is: " << position_d_.transpose() <<  std::endl;  // Debugging */
  //std::cout << "position_d_target update_stiffness_and_references is: " << position_d_target_.transpose() <<  std::endl;  // Debugging
}



// ---------------------------------------------------------------------------
// arrayToMatrix for 7D vectors
// ---------------------------------------------------------------------------
void AdmittanceController::arrayToMatrix(const std::array<double,7>& inputArray, Eigen::Matrix<double,7,1>& resultMatrix)
{
 for(long unsigned int i = 0; i < 7; ++i){
     resultMatrix(i,0) = inputArray[i];
   }
}



// ---------------------------------------------------------------------------
// arrayToMatrix for 6D vectors
// ---------------------------------------------------------------------------
void AdmittanceController::arrayToMatrix(const std::array<double,6>& inputArray, Eigen::Matrix<double,6,1>& resultMatrix)
{
 for(long unsigned int i = 0; i < 6; ++i){
     resultMatrix(i,0) = inputArray[i];
   }
}



// ---------------------------------------------------------------------------
// saturateTorqueRate
// ---------------------------------------------------------------------------
Eigen::Matrix<double, 7, 1> AdmittanceController::saturateTorqueRate(
  const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
  const Eigen::Matrix<double, 7, 1>& tau_J_d_M) {  
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
  double difference = tau_d_calculated[i] - tau_J_d_M[i];
  tau_d_saturated[i] =
         tau_J_d_M[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
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
controller_interface::InterfaceConfiguration AdmittanceController::command_interface_configuration() const {
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
controller_interface::InterfaceConfiguration AdmittanceController::state_interface_configuration()
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
CallbackReturn AdmittanceController::on_init() {
   auto_declare<std::string>("arm_id", "");
   UserInputServer input_server_obj(&position_d_target_, &rotation_d_target_, &K, &D, &T);
   std::thread input_thread(&UserInputServer::main, input_server_obj, 0, nullptr);
   input_thread.detach();
   real_pose_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/real_pose", rclcpp::QoS(10));
   return CallbackReturn::SUCCESS;
}



// ---------------------------------------------------------------------------
// on_configure
// ---------------------------------------------------------------------------
CallbackReturn AdmittanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
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
    // The robot_description parameter contains the URDF as a string.
    // The buildModelFromXML function parses the URDF and initializes the Pinocchio model.
    pinocchio::urdf::buildModelFromXML(robot_description, model_);
    data_ = pinocchio::Data(model_);
    RCLCPP_INFO(get_node()->get_logger(), "Pinocchio model parsed successfully.");
  
    // Set the end-effector frame ID
    // This frame is used for Cartesian impedance control.
    end_effector_frame_id_ = model_.getFrameId("fr3_hand");
    RCLCPP_INFO(get_node()->get_logger(), "Pinocchio model loaded successfully.");
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
CallbackReturn AdmittanceController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
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
controller_interface::CallbackReturn AdmittanceController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  // franka_robot_model_->release_interfaces();
  RCLCPP_INFO(get_node()->get_logger(), "Controller deactivated.");
  return CallbackReturn::SUCCESS;
}



// ---------------------------------------------------------------------------
// convertToStdArray
// ---------------------------------------------------------------------------
std::array<double, 6> AdmittanceController::convertToStdArray(const geometry_msgs::msg::WrenchStamped& wrench) {
    std::array<double, 6> result;
    result[0] = wrench.wrench.force.x;
    result[1] = wrench.wrench.force.y;
    result[2] = wrench.wrench.force.z;
    result[3] = wrench.wrench.torque.x;
    result[4] = wrench.wrench.torque.y;
    result[5] = wrench.wrench.torque.z;
    return result;
}


/*
// ---------------------------------------------------------------------------
// topic_callback
// ---------------------------------------------------------------------------
void AdmittanceController::topic_callback(const std::shared_ptr<franka_msgs::msg::FrankaRobotState> msg) {
  O_F_ext_hat_K = convertToStdArray(msg->o_f_ext_hat_k);
  arrayToMatrix(O_F_ext_hat_K, O_F_ext_hat_K_M);
}
*/

// ---------------------------------------------------------------------------
// reference_pose_callback
// ---------------------------------------------------------------------------
void AdmittanceController::reference_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    // Handle the incoming pose message
    std::cout << "received reference posistion as " <<  msg->position.x << ", " << msg->position.y << ", " << msg->position.z << std::endl;
    position_d_target_ << msg->position.x, msg->position.y, msg->position.z;
    orientation_d_target_.coeffs() << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
    // You can add more processing logic here
    // Update x_d to reflect the new reference poses
/*     x_d.head(3) = position_d_target_;  // New target position
    x_d.tail(3) << msg->orientation.x, msg->orientation.y, msg->orientation.z;  // New target orientation */
}



// ---------------------------------------------------------------------------
// updateJointStates
// ---------------------------------------------------------------------------
void AdmittanceController::updateJointStates() {
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
controller_interface::return_type AdmittanceController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {  
  updateJointStates();

  // --- Allow one initialization cycle before applying controller torques --
  if (!first_valid_joint_state_received_) {
    RCLCPP_INFO_ONCE(rclcpp::get_logger("admittance_controller"), "Waiting for first valid joint state...");
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
  Eigen::VectorXd gravity = pinocchio::computeGeneralizedGravity(model_, data_, q_);
  coriolis = dynamic_torques - gravity;

  pseudoInverse(jacobian.topLeftCorner(6,7).transpose(), jacobian_transpose_pinv);
  pseudoInverse(jacobian.topLeftCorner(6,7), jacobian_pinv); 

  double damping = 1e-6;
  Lambda = (jacobian.topLeftCorner(6,7) * M.topLeftCorner(7,7).inverse() * jacobian.topLeftCorner(6,7).transpose() + damping * IDENTITY).inverse();
  updateJointStates(); 
  Theta = Lambda;
  D = 2.05 * K.cwiseSqrt() * Lambda.diagonal().cwiseSqrt().asDiagonal(); // Admittance control law to compute desired trajectory
  w = jacobian * dq_; // cartesian velocity
  Eigen::Affine3d transform;
  transform.linear() = data_.oMf[end_effector_frame_id_].rotation();  // Extract rotation
  transform.translation() = data_.oMf[end_effector_frame_id_].translation();  // Extract translation
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  //Force Updates
  F_ext = 0.001 * F_ext + 0.999 * O_F_ext_hat_K_M; // noFiltering
  F_ext.head(3) = -F_ext.head(3);
  
  Eigen::Matrix<double, 6, 1> virtual_error = Eigen::MatrixXd::Zero(6, 1); 
  //std::cout << "Error outer loop is: " << error.transpose() <<  std::endl;
  // Now, align error.tail(3) to use virtual_error like position error
  // You want to use the virtual_error for rotational error (tail):
  // Set current state  
 

  // Convert x_d.tail(3) (Euler angles) to a quaternion
  x_d_orientation_quat_ = Eigen::AngleAxisd(x_d.tail(3)(0), Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(x_d.tail(3)(1), Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(x_d.tail(3)(2), Eigen::Vector3d::UnitZ());

  // Ensure the quaternions are in the same hemisphere
  if (orientation_d_.coeffs().dot(x_d_orientation_quat_.coeffs()) < 0.0) {
      x_d_orientation_quat_.coeffs() << -x_d_orientation_quat_.coeffs();
  }
    // Calculate the virtual error in quaternions
  Eigen::Quaterniond virtual_error_quat = orientation_d_.inverse() * x_d_orientation_quat_;
  // Extract the virtual orientation error as a 3D vector (imaginary part for small-angle approximation)
  // Transform the error into the desired orientation frame
  virtual_error.tail(3) << virtual_error_quat.x(), virtual_error_quat.y(), virtual_error_quat.z();
  virtual_error.tail(3) << -x_d_orientation_quat_.toRotationMatrix() * virtual_error.tail(3);

  //virtual_error.tail(3) = -x_d_orientation_quat_.toRotationMatrix() * virtual_error_quat.vec(); // vec for rotational part
  virtual_error.head(3) = x_d.head(3) - position_d_; //linear error
  // Calculate the desired acceleration using the impedance control laws (for oriquentation part only)
  x_ddot_d = Lambda.inverse() * (0 * F_ext - D * x_dot_d - K * virtual_error);
  // Integrate once to get velocities
  x_dot_d += x_ddot_d * dt;
  // Calculate the angle and axis for the quaternion rotation
  Eigen::Vector3d angular_displacement = x_dot_d.tail(3) * dt; // Angular displacement vector
  // Create the rotation increment quaternion
  Eigen::Quaterniond rotation_increment = Eigen::Quaterniond(Eigen::AngleAxisd(angular_displacement.norm(), angular_displacement.normalized()));
  // Update the orientation quaternion
  x_d_orientation_quat_ = (rotation_increment * x_d_orientation_quat_).normalized(); // Normalize to avoid drift
  //Convert final x_d_orientation_quat_ back to Euler angles if needed
  x_d.tail(3) = x_d_orientation_quat_.toRotationMatrix().eulerAngles(0, 1, 2);
  x_d.head(3) += x_dot_d.head(3) * dt; 

  //inner PID position control loop
  //get new inner positional loop error
  // normalize the quaternion before calculating the error

  //error is overwritten
   // compute cartesian error
  error.head(3) << position - x_d.head(3);
  if (x_d_orientation_quat_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation.inverse() * x_d_orientation_quat_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail(3) << -transform.rotation() * error.tail(3);

  double position_error_norm = error.head(3).norm();
  double geodesic_error = 2.0 * std::acos(std::abs(error_quaternion.w()));
  double manipulability = sqrt((jacobian.topLeftCorner(6,7) * jacobian.topLeftCorner(6,7).transpose()).determinant());

  // --- Publish real pose for visualization ---
  Eigen::Vector3d euler = transform.rotation().eulerAngles(0, 1, 2);
  std_msgs::msg::Float64MultiArray real_pose_msg;
  real_pose_msg.data = {
    position[0],
    position[1],
    position[2],
    euler[0],
    euler[1],
    euler[2],
    position_error_norm,
    geodesic_error,
    manipulability
  };
  real_pose_pub_->publish(real_pose_msg);

  
  // change the PID control depending on the control mode
  switch (control_mode) {
    case POSITION_CONTROL:
      F_admittance = - Kp * error - Kd * w; // position control, chagning Kp here doesn't have any influence on the compliance, only on the accuracy
      break;

    case VELOCITY_CONTROL:
      F_admittance = - Kd * (w - x_dot_d); // velocity control
      break;
  }

  // Force control and filtering
  N = (Eigen::MatrixXd::Identity(7, 7) - jacobian_pinv * jacobian.topLeftCorner(6,7)); // Nullspace projection matrix
  
  Eigen::VectorXd tau_nullspace(7), tau_d(7);
  tau_nullspace << N * (nullspace_stiffness_ * config_control * (q_d_nullspace_ - q_) - //if config_control = true we control the whole robot configuration
                       (2.0 * sqrt(nullspace_stiffness_)) * dq_);  // if config control ) false we don't care about the joint position

  calculate_tau_friction(); //Gets friction forces for current state
  tau_admittance = jacobian.topLeftCorner(6,7).transpose() * Sm * (F_admittance /*+ F_repulsion + F_potential*/);
  auto tau_total = (tau_admittance + tau_nullspace + coriolis + tau_friction /*+ gravity*/); //add nullspace, coriolis, and gravity components to desired torque
  tau_d << tau_total;
  tau_d << saturateTorqueRate(tau_d, tau_J_d_M);  // Saturate torque rate to avoid discontinuities
  tau_J_d_M = tau_d;

  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }

  
  
  if (outcounter % 1000 == 0){
    std::cout << "-------------------------------------------------------------------------------------" << std::endl;
    //std::cout << "F_ext_robot [N]" << std::endl;
    //std::cout << O_F_ext_hat_K << std::endl;
    //std::cout << O_F_ext_hat_K_M << std::endl;
    //std::cout << Lambda*Theta.inverse() << std::endl;
    std::cout << "torques: " << tau_d.transpose() << std::endl;
    //std::cout << "--------" << std::endl;
    //std::cout << tau_nullspace << std::endl;
    //std::cout << "--------" << std::endl;
    //std::cout << "--------" << std::endl;
    //std::cout << coriolis << std::endl;
    //std::cout << "Inertia scaling [m]: " << std::endl;
    //std::cout << T << std::endl;
    
    std::cout << "Lambda: " << std::endl;
    std::cout << Lambda << std::endl;
    std::cout << "jacobian: " << std::endl;
    std::cout << jacobian.topLeftCorner(6,7) << std::endl;
    //std::cout << "External Force is: " << F_ext.transpose() <<  std::endl;
    //std::cout << "Kp multiplier is: " << Kp_multiplier <<  std::endl;
    //std::cout << "Kp is: " << Kp_multiplier <<  std::endl;
    //std::cout << "Current orientation: " << orientation.coeffs().transpose() << std::endl;
    //std::cout << "Target orientation: " << orientation_d_target_.toRotationMatrix().eulerAngles(0, 1, 2).transpose() << std::endl; 
    //std::cout << "x_d rotation is: " << x_d_orientation_quat_.toRotationMatrix().eulerAngles(0, 1, 2).transpose() <<  std::endl;
    
    //std::cout << "Error quaternion: " << error_quaternion.coeffs().transpose() << std::endl;
    std::cout << "joint positions: " << q_.transpose() << std::endl;
    std::cout << "position: " << position.transpose() << std::endl;
    std::cout << "orientation: " << orientation << std::endl;
    std::cout << "position error is: " << error.head(3).transpose() <<  std::endl;
    //std::cout << "Elapsed time is: " << elapsed_time <<  std::endl;
    //std::cout << "Desired Acceleration is: " << x_ddot_d.transpose() <<  std::endl;
    //std::cout << "Desired Velocity is: " << x_dot_d.transpose() <<  std::endl;
    //std::cout << "X desired is: " << x_d.transpose() <<  std::endl;
    //std::cout << "position target is: " << position_d_target_.transpose() <<  std::endl;
    //std::cout << "position_d is: " << position_d_.transpose() <<  std::endl;
    //std::cout << "Inertia is : " << Lambda <<  std::endl;
    //std::cout << "--------------------------------------------------" <<  std::endl;
    //std::cout << "tau friction is " << tau_friction.transpose() << std::endl;
    //std::cout << "tau desired is " << tau_d.transpose() << std::endl;
    //std::cout << "Control mode is: " << control_mode <<  std::endl; */
    //std::cout << "F admittance is: " << F_admittance.transpose() <<  std::endl;
    //std::cout << "Orientation Error is: " << error.tail(3).transpose() <<  std::endl; 
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
PLUGINLIB_EXPORT_CLASS(pdz_controller_library::AdmittanceController,
                       controller_interface::ControllerInterface)