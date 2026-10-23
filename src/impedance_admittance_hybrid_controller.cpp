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

#include <pdz_controller_library/impedance_admittance_hybrid_controller.hpp>
#include <pdz_controller_library/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <vector>

#include <iostream>
#include <fstream> 


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
// ImpedanceAdmittanceHybridController
// ---------------------------------------------------------------------------
ImpedanceAdmittanceHybridController::ImpedanceAdmittanceHybridController()  { //if error, delete node
  elapsed_time = 0.0;
  Kp_multiplier = 1; // Initial multiplier for Kp
  Kp = Kp * Kp_multiplier;  // increasing Kp from 0.1 to 1 made robot far less compliant
  control_mode = POSITION_CONTROL; // sets control mode
  //input_control_mode = TARGET_POSITION; // sets position control mode
  D =  2* K.cwiseSqrt(); // set critical damping from the get go
  Kd = 2 * Kp.cwiseSqrt();
}


/*
//to be added in hybrid_controller.hpptions are copied from the previous hybrid_control_naive package. To be aware that the
//switching process might have problems with continuity
void ImpedanceAdmittanceHybridController::topic_callback(const std::shared_ptr<franka_msgs::msg::FrankaRobotState> msg) {
  O_F_ext_hat_K = convertToStdArray(msg->o_f_ext_hat_k);
  arrayToMatrix(O_F_ext_hat_K, O_F_ext_hat_K_M); 
}
*/


// ---------------------------------------------------------------------------
// calculating_environmental_force_norm
// ---------------------------------------------------------------------------
void ImpedanceAdmittanceHybridController::calculating_environmental_force_norm() {
    //F_norm_current = sqrt(pow(O_F_ext_hat_K[0], 2) + pow(O_F_ext_hat_K[1], 2) + pow(O_F_ext_hat_K[2], 2));
    F_norm_current = F_ext.head(3).norm();
}// maybe t_f_ext_hat_k...idk



// ---------------------------------------------------------------------------
// calculating_position_norm
// ---------------------------------------------------------------------------
void ImpedanceAdmittanceHybridController::calculating_position_norm() {
    //std::array<double, 16> pos_end = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
    X_norm_current = error.head(3).norm();
} 



// ---------------------------------------------------------------------------
// update_environmental_stiffness
// ---------------------------------------------------------------------------
void ImpedanceAdmittanceHybridController::update_environmental_stiffness(){ //calculation the environmental stiffness
        /*F_env[0] = F_env[1];
        X_env[0] = X_env[1];
        

        F_env[1] = F_norm_current;
        X_env[1] = X_norm_current;
        //only a easy moving average
        


        //k_e = (F_env[1] - F_env[0])/(X_env[1] - X_env[0] + 0.001);
        k_e = F_env[1]/ (X_env[1]+ 0.001);

        */


        k_e = error.head(3).norm();

        if(k_e < 0){
          k_e = -k_e;
        }

        /*k_e_history.insert(k_e_history.begin(),k_e);
        if(k_e_history.size() > 5){
          k_e_history.pop_back();
        }
        
        k_e = std::accumulate(k_e_history.begin(), k_e_history.end(), 0.0) / k_e_history.size();*/
        //k_e = F_norm_current / X_norm_current;
}



// ---------------------------------------------------------------------------
// calculating_n
// ---------------------------------------------------------------------------
void ImpedanceAdmittanceHybridController::calculating_n(){
        if(k_e <= k_e_adm){
            n = 1.0; 
        }

        else if (k_e >= k_e_imp){
            n = 0.0;
        }
        else {
            n = 1.0 * (k_e_imp - k_e)/(k_e_imp - k_e_adm);
        }
} //n = 1 => total admittance; n = 0 => total impedance


/*
// ---------------------------------------------------------------------------
// write_to_file
// ---------------------------------------------------------------------------
void write_to_file(const std::string& filename, double value) { //output n and F_ext to external file
    std::ofstream output_file(filename, std::ios::app);
    if (!output_file) {
        std::cerr << "Failed to open the file." << std::endl;
        return;
    }
    output_file << value << std::endl;
    output_file.close();
}
*/



// ---------------------------------------------------------------------------
// schmitt_trigger
// ---------------------------------------------------------------------------
void ImpedanceAdmittanceHybridController::schmitt_trigger(){
  if(schmitt_adm == true && k_e >= k_e_imp){
    schmitt_adm = false;
  }
  else if(schmitt_adm == false && k_e <= k_e_adm){
    schmitt_adm = true;
  }
}



// ---------------------------------------------------------------------------
// update_stiffness_and_references
// ---------------------------------------------------------------------------
void ImpedanceAdmittanceHybridController::update_stiffness_and_references(){
  //target by filtering
  /** at the moment we do not use dynamic reconfigure and control the robot via D, K and T **/
  //K = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * K;
  //D = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * D;
  nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  //std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  F_contact_des = 0.05 * F_contact_target + 0.95 * F_contact_des;
}



// ---------------------------------------------------------------------------
// arrayToMatrix for 7D vectors
// ---------------------------------------------------------------------------
void ImpedanceAdmittanceHybridController::arrayToMatrix(const std::array<double,7>& inputArray, Eigen::Matrix<double,7,1>& resultMatrix)
{
 for(long unsigned int i = 0; i < 7; ++i){
     resultMatrix(i,0) = inputArray[i];
   }
}



// ---------------------------------------------------------------------------
// arrayToMatrix for 6D vectors
// ---------------------------------------------------------------------------
void ImpedanceAdmittanceHybridController::arrayToMatrix(const std::array<double,6>& inputArray, Eigen::Matrix<double,6,1>& resultMatrix)
{
 for(long unsigned int i = 0; i < 6; ++i){
     resultMatrix(i,0) = inputArray[i];
   }
}



// ---------------------------------------------------------------------------
// saturateTorqueRate
// ---------------------------------------------------------------------------
Eigen::Matrix<double, 7, 1> ImpedanceAdmittanceHybridController::saturateTorqueRate(
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
controller_interface::InterfaceConfiguration ImpedanceAdmittanceHybridController::command_interface_configuration() const {
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
controller_interface::InterfaceConfiguration ImpedanceAdmittanceHybridController::state_interface_configuration()
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
CallbackReturn ImpedanceAdmittanceHybridController::on_init() {
   UserInputServer input_server_obj(&position_d_target_, &rotation_d_target_, &K, &D, &T);
   std::thread input_thread(&UserInputServer::main, input_server_obj, 0, nullptr);
   input_thread.detach();
   real_pose_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/real_pose", rclcpp::QoS(10));
  RCLCPP_INFO(get_node()->get_logger(), "on_init completed successfully.");
   return CallbackReturn::SUCCESS;
}



// ---------------------------------------------------------------------------
// on_configure
// ---------------------------------------------------------------------------
CallbackReturn ImpedanceAdmittanceHybridController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
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
CallbackReturn ImpedanceAdmittanceHybridController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
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
controller_interface::CallbackReturn ImpedanceAdmittanceHybridController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(get_node()->get_logger(), "Controller deactivated.");
  return CallbackReturn::SUCCESS;
}



// ---------------------------------------------------------------------------
// convertToStdArray
// ---------------------------------------------------------------------------
std::array<double, 6> ImpedanceAdmittanceHybridController::convertToStdArray(const geometry_msgs::msg::WrenchStamped& wrench) {
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
// reference_pose_callback
// ---------------------------------------------------------------------------
void ImpedanceAdmittanceHybridController::reference_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) //desired position , to be changed
{
    // Handle the incoming pose message
    std::cout << "received reference posistion as " <<  msg->position.x << ", " << msg->position.y << ", " << msg->position.z << std::endl;
    position_d_target_ << msg->position.x, msg->position.y, msg->position.z;
    orientation_d_target_.coeffs() << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
    //orientation_d_target_.normalize();




    // You can add more processing logic here
    // Update x_d to reflect the new reference poses
/*     x_d.head(3) = position_d_target_;  // New target position
    x_d.tail(3) << msg->orientation.x, msg->orientation.y, msg->orientation.z;  // New target orientation */
}



// ---------------------------------------------------------------------------
// updateJointStates
// ---------------------------------------------------------------------------
void ImpedanceAdmittanceHybridController::updateJointStates() {
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
controller_interface::return_type ImpedanceAdmittanceHybridController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {  
  #define DEBUG_CHECKPOINT(msg) \
  RCLCPP_ERROR_ONCE( \
    rclcpp::get_logger("hybrid_debug"), \
    "[HYBRID DEBUG] %s", msg)

  DEBUG_CHECKPOINT("01: Entered update");

  updateJointStates();
  DEBUG_CHECKPOINT("02: updateJointStates completed");

  if (!first_valid_joint_state_received_) {
    RCLCPP_INFO_ONCE(
        rclcpp::get_logger("impedance_admittance_hybrid_controller"),
        "Waiting for first valid joint state...");

    first_valid_joint_state_received_ = true;

    for (size_t i = 0; i < 7; ++i) {
      command_interfaces_[i].set_value(0.0);
    }

    DEBUG_CHECKPOINT("03: Initialization return");
    return controller_interface::return_type::OK;
  }

  DEBUG_CHECKPOINT("04: Passed initialization guard");
  
  DEBUG_CHECKPOINT("05: Before rnea");

  Eigen::VectorXd dynamic_torques = pinocchio::rnea(model_, data_, q_, dq_, Eigen::VectorXd::Zero(model_.nv));

  DEBUG_CHECKPOINT("06: After rnea");

  M = pinocchio::crba(model_, data_, q_);

  DEBUG_CHECKPOINT("07: After crba");

  pinocchio::computeFrameJacobian(model_, data_, q_, end_effector_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);

  DEBUG_CHECKPOINT("08: After computeFrameJacobian");

  pinocchio::forwardKinematics(model_, data_, q_);

  DEBUG_CHECKPOINT("09: After forwardKinematics");

  pinocchio::updateFramePlacements(model_, data_);

  DEBUG_CHECKPOINT("10: After updateFramePlacements");

  Eigen::VectorXd gravity =
      pinocchio::computeGeneralizedGravity(model_, data_, q_);

  DEBUG_CHECKPOINT("11: After computeGeneralizedGravity");

  coriolis = dynamic_torques - gravity;

  DEBUG_CHECKPOINT("12: After coriolis calculation");

  DEBUG_CHECKPOINT("13: Before transform construction");

  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.linear() = data_.oMf[end_effector_frame_id_].rotation();
  transform.translation() = data_.oMf[end_effector_frame_id_].translation();

  Eigen::Vector3d position = transform.translation();
  Eigen::Quaterniond orientation(transform.rotation());

  DEBUG_CHECKPOINT("14: After transform construction");

  orientation_d_target_ =
      Eigen::AngleAxisd(rotation_d_target_[0], Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(rotation_d_target_[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rotation_d_target_[2], Eigen::Vector3d::UnitZ());

  DEBUG_CHECKPOINT("15: After target orientation");

  updateJointStates();
  calculating_environmental_force_norm();
  calculating_position_norm();
  update_environmental_stiffness();
  calculating_n();
  schmitt_trigger();

  DEBUG_CHECKPOINT("16: After helper calculations");
  
  Lambda = (jacobian.topLeftCorner(6,7) * M.topLeftCorner(7,7).inverse() * jacobian.topLeftCorner(6,7).transpose()).inverse();
  Theta = Lambda;
  D = 2.05 * K.cwiseSqrt() * Lambda.diagonal().cwiseSqrt().asDiagonal();

  DEBUG_CHECKPOINT("17: After Lambda/Theta/D calculation");

  w = jacobian * dq_;

  DEBUG_CHECKPOINT("18: After Cartesian velocity");

  acceleration = (w - w_last) / dt;
  w_last = w;

  error_dot = (error - error_last) / dt;
  error_last = error;

  DEBUG_CHECKPOINT("19: After acceleration and error derivatives");

  DEBUG_CHECKPOINT("20: Error calculations");

  error.head(3) << position - position_d_;
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail(3) << -transform.rotation() * error.tail(3);
  
  DEBUG_CHECKPOINT("21: Force filterin");

  //Force Updates
  F_ext = 0.9 * F_ext + 0.1 * O_F_ext_hat_K_M; // noFiltering
  F_ext.head(3) = -F_ext.head(3);

  DEBUG_CHECKPOINT("22: Quaternion calculations");

 // Convert x_d.tail(3) (Euler angles) to a quaternion
  Eigen::Matrix<double, 6, 1> virtual_error = Eigen::MatrixXd::Zero(6, 1);
  x_d_orientation_quat_ = Eigen::AngleAxisd(x_d.tail(3)(0), Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(x_d.tail(3)(1), Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(x_d.tail(3)(2), Eigen::Vector3d::UnitZ());


  if (orientation_d_.coeffs().dot(x_d_orientation_quat_.coeffs()) < 0.0) {
      x_d_orientation_quat_.coeffs() << -x_d_orientation_quat_.coeffs();
  }

  DEBUG_CHECKPOINT("23: Virtual error");

  Eigen::Quaterniond virtual_error_quat = (x_d_orientation_quat_.inverse() * orientation_d_);

  // Normalize and check small norm
  if (virtual_error_quat.norm() > 1e-6) {
      virtual_error_quat.normalize();
      // Use angle-axis representation to extract error vector
      Eigen::AngleAxisd angle_axis_virtual_error(virtual_error_quat);
      Eigen::Vector3d rotation_virtual_error_vector = angle_axis_virtual_error.angle() * angle_axis_virtual_error.axis();
      // Transform error into target frame
      virtual_error.tail(3) = -transform.rotation() * rotation_virtual_error_vector;
  } else {
      virtual_error.tail(3).setZero();
  }


  //virtual_error.tail(3) = -x_d_orientation_quat_.toRotationMatrix() * virtual_error_quat.vec(); // vec for rotational part
  virtual_error.head(3) = x_d.head(3) - position_d_; //linear erro

  //F_ext?
  x_ddot_d = Lambda.inverse() * ( /*0 * */ F_ext - D * x_dot_d - K * virtual_error);
  // Integrate once to get velocities
  x_dot_d += x_ddot_d * dt;
  // Calculate the angle and axis for the quaternion rotation
  Eigen::Vector3d angular_displacement = x_dot_d.tail(3) * dt; // Angular displacement vector
  // Create the rotation increment quaternion
  Eigen::Quaterniond rotation_increment = Eigen::Quaterniond(Eigen::AngleAxisd(angular_displacement.norm(), angular_displacement.normalized()));

  x_d_orientation_quat_ = (rotation_increment * x_d_orientation_quat_).normalized(); // Normalize to avoid drift
  //Convert final x_d_orientation_quat back to Euler angles if needed
  x_d.tail(3) = x_d_orientation_quat_.toRotationMatrix().eulerAngles(0, 1, 2);
  x_d.head(3) += x_dot_d.head(3) * dt; 
  //x_d.head(3) << 0,0,0;
  error_pd.head(3) << position - x_d.head(3);
  //error_pd.head(3) <<0,0,0;
  if (x_d_orientation_quat_.coeffs().dot(orientation.coeffs()) < 0.0) {
  orientation.coeffs() << -orientation.coeffs();
  }
    
  //error here
  
  DEBUG_CHECKPOINT("24: Pose data gathering");
  //x - x_d
  Eigen::Quaterniond error_quaternion_current (orientation.inverse() * x_d_orientation_quat_);
  error_pd.tail(3) << error_quaternion_current.x(), error_quaternion_current.y(), error_quaternion_current.z();
  error_pd.tail(3) << -transform.rotation() * error_pd.tail(3); 

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

  DEBUG_CHECKPOINT("25: Pose data published");
  
  DEBUG_CHECKPOINT("26: Entering switch");

  switch (mode_) {
    case 1:
      {
      if(counter_ == 0){
        n_current = n;
      } //make sure that within every loop the n is constant

      if(counter < (20 - n_current * 20)) { // Impedance control

        if (smooth_adm_imp == 1){
          pos_d_switching.head(3) << position_d_;
          pos_d_switching.tail(3) << orientation_d_.toRotationMatrix().eulerAngles(0, 1, 2);

          /*pos_switching_imp = K.inverse() * (Kp * error_pd + Kd *w - D*w) + pos_d_switching;
          
          error.head(3) << pos_switching_imp.head(3) - position_d_;
          orientation = Eigen::AngleAxisd(pos_switching_imp[4], Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pos_switching_imp[5], Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(pos_switching_imp[6], Eigen::Vector3d::UnitX());
          Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
          error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
          error.tail(3) << -transform.rotation() * error.tail(3); */

          error = K.inverse() * (Kp * error_pd + Kd *w_last - D*w_last);


          //error = Lambda.inverse() * (Kp * error_pd + Kd * w - D * w);

          smooth_adm_imp = 0;
        } 
        F_impedance_current = -1 * (D * (jacobian * dq_) + K /*K_imp*/ * error);
        //F_impedance = (1 - n_current) * F_impedance_current + n_current * F_impedance_last; 
        desired_F = F_impedance_current; //desired_F for controlling the settling time, no impact on output
        //double blend_weight = 0.5 * (1 - cos(n_current * M_PI));
        double blend_weight = 0.9;
        F_output = blend_weight * F_impedance_current + (1 - blend_weight) * F_last;
        //delta_F_impedance = F_impedance_current - F_impedance_last;
        //delta_F_impedance = delta_F_impedance.cwiseMax(-max_F_impedance).cwiseMin(max_F_impedance);
        //F_impedance = F_impedance_last + delta_F_impedance;
        F_last = F_output;
        ++counter;
        ++counter_;
        flag_imp = true;
        smooth_imp_adm = 1;

      } else if(counter < 20) { // Admittance control

        if(smooth_imp_adm == 1){

          pos_switching_adm.head(3) << position;
          pos_switching_adm.tail(3) << orientation.toRotationMatrix().eulerAngles(0, 1, 2);
        

          //x - x_d
          /*error_pd.head(3) << position - x_d.head(3);
          Eigen::Quaterniond x_d_quat = Eigen::AngleAxisd(x_d[4], Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(x_d[5], Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(x_d[6], Eigen::Vector3d::UnitX());
          Eigen::Quaterniond error_quaternion_x_d(orientation.inverse()*x_d_quat );
          error_pd.tail(3) << error_quaternion_x_d.x(), error_quaternion_x_d.y(), error_quaternion_x_d.z();
          error_pd.tail(3) << -transform.rotation() * error_pd.tail(3); */

          error_pd = Kp.inverse() * (D * w_last + K /*K_imp*/ * error - Kd * w_last);
          x_d = pos_switching_adm - error_pd;
          x_dot_d = -Kp.inverse() * (D * acceleration + K * w_last /*- position_d_, which is 0 in test case*/ - Kd * acceleration) + w;

          /*
          pos_switching_adm.head(3) << position;
          pos_switching_adm.tail(3) << orientation.toRotationMatrix().eulerAngles(0, 1, 2);
          Eigen::Matrix<double, 6, 1> A = F_ext - K * error - D * w;
          x_d = pos_switching_adm + Kp.inverse() * (Kd * w- F_ext + Lambda.inverse() * A);
          error_pd.head(3) << position - x_d.head(3);
          Eigen::Quaterniond x_d_quat = Eigen::AngleAxisd(x_d[4], Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(x_d[5], Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(x_d[6], Eigen::Vector3d::UnitX());
          Eigen::Quaterniond error_quaternion_x_d(orientation.inverse() * x_d_quat );
          error_pd.tail(3) << error_quaternion_x_d.x(), error_quaternion_x_d.y(), error_quaternion_x_d.z();
          error_pd.tail(3) << -transform.rotation() * error_pd.tail(3);
          virtual_error.head(3) = x_d.head(3) - position_d_;
          */
          //x_dot_d.setZero();//naive Initialization 
        
          smooth_imp_adm = 0;
        }
          
        
        F_admittance_current  = - Kp * error_pd - Kd * w_last;
        
        //F_impedance = n_current * F_impedance_current + (1 - n_current) * F_impedance_last;
        desired_F = F_admittance_current;
        double blend_weight = 0.9;
        F_output = blend_weight * F_admittance_current + (1 - blend_weight) * F_last;

      
        //delta_F_impedance = F_admittance_current - F_impedance_last;
        //delta_F_impedance = delta_F_impedance.cwiseMax(-max_F_impedance).cwiseMin(max_F_impedance);
        //F_impedance = F_impedance_last + delta_F_impedance;
        F_last = F_output;
        ++counter;
        ++counter_;
        flag_imp = false;
        smooth_adm_imp = 1;
      } else {
        counter = 0;
        counter_ = 0;
      }
      //F_output.tail(3) = Eigen::Vector3d(0, 0, 0);//test


      } 
    break;

    case 2:{ //schmitt trigger
      if(schmitt_adm == true){
        Eigen::Matrix<double, 6, 1> F_admittance_current = - Kp * error_pd - Kd * w_last;
        desired_F = F_admittance_current;
        double blend_weight = 0.9;
        F_output = blend_weight * F_admittance_current + (1 - blend_weight) * F_last;
        F_last = F_output;
        flag_imp = false;
      }
      else{
        Eigen::Matrix<double, 6, 1> F_impedance_current = -1 * (D * (jacobian * dq_) + K /*K_imp*/ * error);
        //F_impedance = (1 - n_current) * F_impedance_current + n_current * F_impedance_last; 
        desired_F = F_impedance_current; //desired_F for controlling the settling time, no impact on output
        double blend_weight = 0.9;
        F_output = blend_weight * F_impedance_current + (1 - blend_weight) * F_last;
        F_last = F_output;
        flag_imp = true;
      }
    }
    break;

    case 3:{
      F_impedance_current = -1 * (D * (jacobian * dq_) + K /*K_imp*/ * error); //imp 
      desired_F = F_impedance_current; //desired_F for controlling the settling time, no impact on output
      //double blend_weight = 0.9;
      //F_output = blend_weight * F_impedance_current + (1 - blend_weight) * F_last;
      F_output = F_impedance_current;
      F_last = F_output;
    }
    break;

    case 4:{
      F_admittance_current = - Kp * error_pd - Kd * w; //adm
      desired_F = F_admittance_current;
      //double blend_weight = 0.9;
      //F_output = blend_weight * F_admittance_current + (1 - blend_weight) * F_last;
      F_output = F_admittance_current;
      F_last = F_output;
    }
    break;
    
    default:
      break;
  }

  DEBUG_CHECKPOINT("27: Switch completed");

  DEBUG_CHECKPOINT("28: torque calculation");

  //F_impedance = - Kp * error_pd  - Kd * w;
    
  //in admittance, F_ext.head(3) = -F_ext.head(3), why?
 // F_ext = 0.9 * F_ext + 0.1 * O_F_ext_hat_K_M; //Filtering 
  /*I_F_error += dt * Sf* (F_contact_des - F_ext);
  F_cmd = Sf*(0.4 * (F_contact_des - F_ext)orientation_d_transpose_pinv); */
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_impedance(7);
  pseudoInverse(jacobian.topLeftCorner(6,7).transpose(), jacobian_transpose_pinv);

  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.topLeftCorner(6,7).transpose() * jacobian_transpose_pinv) *
                    (nullspace_stiffness_ * config_control * (q_d_nullspace_ - q_) - //if config_control = true we control the whole robot configuration
                    (2.0 * sqrt(nullspace_stiffness_)) * dq_);  // if config control ) false we don't care about the joint position

  //calculate_tau_friction();
  tau_impedance = jacobian.transpose() * Sm * (F_output /*+ F_repulsion + F_potential*/)  /* + jacobian.transpose() * Sf * F_cmd*/;
  auto tau_d_placeholder = tau_impedance + tau_nullspace + coriolis  /*tau_friction*/; //add nullspace and coriolis components to desired torque
  tau_d << tau_d_placeholder;
  tau_d << saturateTorqueRate(tau_d, tau_J_d_M);  // Saturate torque rate to avoid discontinuities
  tau_J_d_M = tau_d;

  DEBUG_CHECKPOINT("29: commanding torques");

  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }

  DEBUG_CHECKPOINT("30: printing log");
  
  if (outcounter % 1000 == 0){

    //std::cout << "loop_duration_imp [ms]: " << std::endl;
    //std::cout << loop_duration_imp << std::endl;
    //std::cout << "Target orientation: " << orientation_d_target_.toRotationMatrix().eulerAngles(0, 1, 2).transpose() << std::endl; //correct
    //std::cout << "x_d rotation is: " << x_d_orientation_quat.toRotationMatrix().eulerAngles(0, 1, 2).transpose() <<  std::endl; //wrong
    //std::cout << "Orientation virtual_error is: " << virtual_error.tail(3).transpose() <<  std::endl; //correct
    //std::cout << "Orientation Error_pd is: " << error.tail(3).transpose() <<  std::endl; //correct
    //std::cout << "Orientation_d_ is: " << orientation_d_.toRotationMatrix().eulerAngles(0, 1, 2).transpose() <<  std::endl;
    //std::cout << "Orientation is: " << orientation.toRotationMatrix().eulerAngles(0, 1, 2).transpose() <<  std::endl;
    //std::cout << "Current n is: " << n <<  std::endl;
    //std::cout << "Current F_norm_current is: " << F_norm_current <<  std::endl;
    //std::cout << "Current X_norm_current is: " << X_norm_current <<  std::endl;
    //std::cout << "Current k_e is: " << k_e <<  std::endl;
    //std::cout << "Current tau_impedance is: " << tau_impedance <<  std::endl;
    //std::cout << "Current counter is: " << counter <<  std::endl;
    //std::cout << "If impedance " << flag_imp <<  std::endl;    
    std::cout << "torques: " << tau_d.transpose() << std::endl;
    std::cout << "position: " << position.transpose() << std::endl;
    std::cout << "orientation: " << orientation << std::endl;
    std::cout << "error: " << error.transpose() << std::endl;
  }
  outcounter++;
  update_stiffness_and_references();
  return controller_interface::return_type::OK;
}
}

// namespace pdz_controller_library
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(pdz_controller_library::ImpedanceAdmittanceHybridController,
                       controller_interface::ControllerInterface)
