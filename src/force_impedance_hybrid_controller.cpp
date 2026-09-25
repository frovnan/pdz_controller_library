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

#include <pdz_controller_library/force_impedance_hybrid_controller.hpp>      // In this file all variables and functions are defined.

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>                                                         // includes the whole Eigen library

namespace {

// the following template allows the following type of code to work:          
// std::cout << name_of_array << std::endl; 
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

ForceImpedanceHybridController::ForceImpedanceHybridController() {}

//this function should change a wrench from base frame to end effector frame
//NO GUARANTEE THAT IT WORKS, IT WASN'T TESTED
void ForceImpedanceHybridController::change_wrench_to_endeffector_frame(Eigen::Matrix<double, 6, 1>& F_ext, 
                                       const Eigen::Matrix<double, 3, 3>& rot_matrix, 
                                       const Eigen::Matrix<double, 3, 1>& position) {
    Eigen::Vector3d force_base = F_ext.head<3>();
    Eigen::Vector3d moment_base = F_ext.tail<3>();
    Eigen::Matrix<double, 3, 3> rot_matrix_inv = rot_matrix.transpose();
    Eigen::Vector3d force_eff = rot_matrix_inv * force_base;
    Eigen::Vector3d moment_eff = rot_matrix_inv * (moment_base - position.cross(force_base));
    F_ext.head<3>() = force_eff;
    F_ext.tail<3>() = moment_eff;
}

//this function should change a wrench from end effector frame to base frame
//NO GUARANTEE THAT IT WORKS, IT WASN'T TESTED
void ForceImpedanceHybridController::change_wrench_to_base_frame(Eigen::Matrix<double, 6, 1>& F_cmd, const Eigen::Matrix<double,3,3>& rot_matrix,
                                                               const Eigen::Matrix<double,3,1>& position /*the position is the translation between the 2 frames*/){
  Eigen::Vector3d force_eff = F_cmd.head<3>();
  Eigen::Vector3d moment_eff = F_cmd.tail<3>();
  Eigen::Vector3d force_base_frame = rot_matrix * force_eff;
  Eigen::Vector3d moment_base_frame = rot_matrix * moment_eff + position.cross(force_base_frame);
  F_cmd.head<3>() = force_base_frame;
  F_cmd.tail<3>() = moment_base_frame;
                                                               }
// calculates the observed forces according to the Bachelor Thesis of Viktor Luba: https://github.com/LucasG2001/MA_Force_Control/tree/final_code_Viktor
void ForceImpedanceHybridController::update_forces(Eigen::Matrix<double, 6, 1>& F_observed, 
                                                Eigen::Matrix<double, 7, 1>& beta_observer, 
                                                Eigen::Matrix<double, 7, 1>& gamma_observer,
                                                Eigen::Matrix<double, 7, 1>& r_observer,
                                                const Eigen::Matrix<double, 7, 7>& K_observer,
                                                const Eigen::Matrix<double, 7, 1>& coriolis,
                                                const Eigen::Matrix<double, 7, 7>& M_dot,
                                                const Eigen::Matrix<double, 7, 1>& dq_,
                                                const Eigen::Matrix<double, 7, 1>& tau_d,
                                                const double& dt,
                                                const Eigen::Matrix<double, 7, 7>& M,
                                                const Eigen::MatrixXd& jacobian_transpose_pinv) {
  beta_observer = coriolis - M_dot * dq_;
  gamma_observer += (tau_d - beta_observer + r_observer) * dt;
  r_observer = K_observer * (M * dq_ - gamma_observer);
  F_observed = jacobian_transpose_pinv * r_observer;
}

//updates the selection matrices Sm and Sf according to the requested wrench
void ForceImpedanceHybridController::adapt_Sm_and_Sf(const int& frame, const Eigen::Matrix<double, 6, 1>& F_contact_target, const Eigen::Matrix<double,3,3>& rot_matrix, 
                        Eigen::Matrix<double, 6, 6>& Sm, Eigen::Matrix<double, 6, 6>& Sf){
  Sm = Eigen::MatrixXd::Zero(6, 6);
  for (int i =0; i<6; ++i){
    if (F_contact_target(i)==0){
      Sm(i,i) = 1;
    }
  }
  
  Sf = IDENTITY - Sm;

  //This part is meant for executing forces in the end effector frame.
  //IT DOES CURRENTLY NOT WORK. Feel free to use it as a foundation to implement a version that works.
  if (frame == 2){
    Eigen::Matrix<double, 3, 3> Sm_top_left = Sm.topLeftCorner(3,3);
    Eigen::Matrix<double, 3, 3> Sm_bottom_right = Sm.bottomRightCorner(3,3);
    Eigen::Matrix<double, 3, 3> Identity_3x3 = Eigen::MatrixXd::Identity(3, 3);
    
    // Calculations according to robot dynamics script chapter 3.9.4 Operational Space Control. The matrices had to to be inverted (= transposed to get it running).
    Sm.topLeftCorner(3,3) = rot_matrix.transpose() * Sm_top_left * rot_matrix;
    Sm.bottomRightCorner(3,3) = rot_matrix.transpose() * Sm_bottom_right * rot_matrix;
    Sf.topLeftCorner(3,3) = rot_matrix.transpose() * (Identity_3x3 - Sm_top_left) * rot_matrix;  
    Sf.bottomRightCorner(3,3) = rot_matrix.transpose() * (Identity_3x3 - Sm_bottom_right) * rot_matrix;
  }

  
}

// Updates the requested position when the requested force gets changed.
// Ensures no abrupt movement of the end effector once the robot should stop exerting forces/torques in/around a certain axis.
void ForceImpedanceHybridController::change_desired_position_if_force_changes(const Eigen::Quaterniond orientation, const Eigen::Vector3d position, const Eigen::Matrix<double, 6, 1>& F_contact_target, 
                        Eigen::Matrix<double, 6, 1>& F_request_old, Eigen::Vector3d& position_d_, Eigen::Quaterniond& orientation_d_, 
                        Eigen::Vector3d& position_d_target_, Eigen::Vector3d& rotation_d_target_/*, bool& keepprinting*/){
  for (int i =0; i<6; ++i){
    if (F_contact_target(i)!= F_request_old(i)){
      position_d_ = position;
      position_d_target_ = position;
      orientation_d_ = orientation;
      Eigen::Matrix3d rot_Matrix_to_transform_to_euler_angles = orientation.toRotationMatrix();
      rotation_d_target_ = rot_Matrix_to_transform_to_euler_angles.eulerAngles(0,1,2);
      F_request_old = F_contact_target;
      break;
    }
  }
}


//Removes the friction from the measured external wrench.
void ForceImpedanceHybridController::get_rid_of_friction_force(const Eigen::MatrixXd& jacobian_transpose_pinv, 
                                                            const Eigen::Matrix<double, 7, 1>& tau_friction, 
                                                            const Eigen::Matrix<double, 6, 1>& O_F_ext_hat_K_M,
                                                            Eigen::Matrix<double, 6, 1>& O_F_ext_hat_K_M_no_friction){
  O_F_ext_hat_K_M_no_friction = O_F_ext_hat_K_M - jacobian_transpose_pinv * tau_friction;
 
}

//Calculates the friction torques acting on each joint
//Taken from the bachelor thesis of Viktor Luba: https://polybox.ethz.ch/index.php/apps/richdocuments/documents.php/public?fileId=3679782949&shareToken=iYj8ALPijKTAC2z
void ForceImpedanceHybridController::calculate_tau_friction(const Eigen::Matrix<double, 7, 1>& dq_, const Eigen::Matrix<double, 7, 1>& tau_impedance,
                                                          const Eigen::Matrix<double, 6, 7>& jacobian, const Eigen::Matrix<double, 6, 6>& Sm,
                                                          const Eigen::Matrix<double, 6, 1>& error, const Eigen::MatrixXd& jacobian_pinv){
    double alpha = 0.01;//constant for exponential filter in relation to static friction moment        
    dq_filtered = alpha* dq_ + (1 - alpha) * dq_filtered; //Filtering dq_ of every joint
    tau_impedance_filtered = alpha*tau_impedance + (1 - alpha) * tau_impedance_filtered; //Filtering tau_impedance
    //Creating and filtering a "fake" tau_impedance with own weights, optimized for friction compensation (else friction compensation would get stronger with higher stiffnesses)
    tau_friction_impedance = jacobian.transpose() * Sm * (-alpha * (D_friction*(jacobian*dq_) + K_friction * error)) + (1 - alpha) * tau_friction_impedance;
    //Creating "fake" dq_, that acts only in the impedance-space, else dq_ in the nullspace also gets compensated, which we do not want due to null-space movement
    N = (Eigen::MatrixXd::Identity(7, 7) - jacobian_pinv * jacobian); //Nullspace matrix 
    dq_imp = dq_filtered - N * dq_filtered;

    f = beta.cwiseProduct(dq_imp) + offset_friction;
    g(4) = (coulomb_friction(4) + (static_friction(4) - coulomb_friction(4)) * exp(-1 * std::abs(dq_imp(4)/dq_s(4))));                    // this lines was never tested before
    g(6) = (coulomb_friction(6) + (static_friction(6) - coulomb_friction(6)) * exp(-1 * std::abs(dq_imp(6)/dq_s(6))));                    // this lines was never tested before
    dz = dq_imp.array() - dq_imp.array().abs() / g.array() * sigma_0.array() * z.array() + 0.025* tau_friction_impedance.array()/*(jacobian.transpose() * K * error).array()*/;
    dz(6) -= 0.02*tau_friction_impedance(6);                                             // k is everywhere 0.025 excet for the last one that's why we do this wired move (last one is only 0.005 thats why we have wired differnce here)
    z = 0.001 * dz + z;
    tau_friction = sigma_0.array() * z.array() + 100 * sigma_1.array() * dz.array() + f.array();  //*100 because values in vector are only 1% of real values
}


//Updates the desired position, orientation and force. It filters them using an expoential filter to make their step response less agressive.
//The commented part was implemented by Curdin Deplazes and not needed for my thesis.
void ForceImpedanceHybridController::update_stiffness_and_references(Eigen::Matrix<double, 6, 1>& F_contact_des){
  //target by filtering
  /** at the moment we do not use dynamic reconfigure and control the robot via D, K and T **/
  //K = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * K;
  //D = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * D;
  nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  //std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);                           // slerp calculates the shortest way to reach desired orient.
  F_contact_des = 0.05 * F_contact_target + 0.95 * F_contact_des;
}

// Converts a 7 dimensional array into a 7x1 Eigen Matrix
void ForceImpedanceHybridController::arrayToMatrix(const std::array<double,7>& inputArray, Eigen::Matrix<double,7,1>& resultMatrix)
{
 for(long unsigned int i = 0; i < 7; ++i){
     resultMatrix(i,0) = inputArray[i];
   }
}

// Converts a 6 dimensional array into a 6x1 Eigen Matrix
void ForceImpedanceHybridController::arrayToMatrix(const std::array<double,6>& inputArray, Eigen::Matrix<double,6,1>& resultMatrix)
{
 for(long unsigned int i = 0; i < 6; ++i){
     resultMatrix(i,0) = inputArray[i];
   }
}

// This function ensures that the torques (tau) aren't changed to quickly
// tau_J_d_M is the calculated tau from 1 iteration before
Eigen::Matrix<double, 7, 1> ForceImpedanceHybridController::saturateTorqueRate( 
  const Eigen::Matrix<double, 7, 1>& tau_d_calculated,                                          
  const Eigen::Matrix<double, 7, 1>& tau_J_d_M) {  
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d_M[i];
    tau_d_saturated[i] =
    tau_J_d_M[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);        // the max(min()) makes sure that the absolute value of the difference 
  }                                                                                        // to the previous value is bellow delta_tau_max
  return tau_d_saturated;
}

//Computes the Pseudo Inverse of the first entry and stores it in the second entry
inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {  //the inline before the void is only to improve the codes performance
  double lambda_ = damped ? 0.2 : 0.0;                                                                // damped == true -> lambda_ = 0.2    damped == false --> lambda_ = 0
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
     S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Begin of setup functions. Not important.

controller_interface::InterfaceConfiguration                                                          // creates the Interface configuration aka names all 
ForceImpedanceHybridController::command_interface_configuration() const {                               // the different joints and stores them in a vector 
  controller_interface::InterfaceConfiguration config;                                                // and sets the interface type to individual
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}


controller_interface::InterfaceConfiguration ForceImpedanceHybridController::state_interface_configuration()
  const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    state_interfaces_config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/position");    // adds names for position and velocity for each joint
    state_interfaces_config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/velocity");
  }

  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    state_interfaces_config.names.push_back(franka_robot_model_name);                                     // adds other interfaces to the config vector
    std::cout << franka_robot_model_name << std::endl;
  }

  const std::string full_interface_name = robot_name_ + "/" + state_interface_name_;

  return state_interfaces_config;
}

//End of setup functions.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Begin of Callback functions. They are only executed once at the start of the controller. For more details refer to: https://polybox.ethz.ch/index.php/apps/richdocuments/documents.php/public?fileId=3688733710&shareToken=iYj8ALPijKTAC2z
//UNLESS YOU WANT TO IMPLEMENT NEW NODES YOU PROBABLY DON'T HAVE TO CHANGE ANYTHING HERE.

CallbackReturn ForceImpedanceHybridController::on_init() {
  
  //start the UserInputServer
   UserInputServer input_server_obj(&position_d_target_,& rotation_d_target_, /*&gripper_command,*/ & K,& D,& T);                // creates or starts the server aka calls constructor of the UserInputServer defined in user_input_server.hpp
   std::thread input_thread(&UserInputServer::main, input_server_obj, 0, nullptr);
   input_thread.detach();                                                                                 // disconnect thread so other thread mustn't wait for 
   
   //start the UserInputForceServer
   UserInputForceServer input_server_obj2(&F_contact_target, &frame);                
   std::thread input_thread2(&UserInputForceServer::main, input_server_obj2, 0, nullptr);
   input_thread2.detach();
/////////// The following 3 lines can be used to start an implemented server:
/////////// Capitalised things must be replaced, things starting with YOUR can be replaced by whatever name you deam appropriate
  //  NAME_OF_SERVER_CLASS YOUR_NAME_FOR_THE_SERVER(&VARIABLE_1, &VARIABLE_2, ...);                
  //  std::thread YOUR_NAME_OF_THIS_THREAD(&NAME_OF_SERVER_CLASS::main, YOUR_NAME_FOR_THE_SERVER, 0, nullptr);
  //  YOUR_NAME_OF_THIS_THREAD.detach();
///////////
   return CallbackReturn::SUCCESS;
}


CallbackReturn ForceImpedanceHybridController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(                   // std::make_unique creates idiotproof pointer
  franka_semantic_components::FrankaRobotModel(robot_name_ + "/" + k_robot_model_interface_name,
                                               robot_name_ + "/" + k_robot_state_interface_name));

  // creates a subscribtion to the topic franka_robot_state_broadcaster/robot_state                                           
  try {
    rclcpp::QoS qos_profile(1); // Depth of the message queue aka does only keep one message all older ones get deleted
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    franka_state_subscriber = get_node()->create_subscription<franka_msgs::msg::FrankaRobotState>(
    "franka_robot_state_broadcaster/robot_state", qos_profile, 
    std::bind(&ForceImpedanceHybridController::topic_callback, this, std::placeholders::_1));
    std::cout << "Succesfully subscribed to robot_state_broadcaster" << std::endl;
  }

  catch (const std::exception& e) {
    fprintf(stderr,  "Exception thrown during publisher creation at configure stage with message : %s \n",e.what());
    return CallbackReturn::ERROR;
    }
  
  // creates a publisher to the topic wrench_topic
  try {
    rclcpp::QoS qos_profile2(1); // Depth of the message queue aka does only keep one message all older ones get deleted
    qos_profile2.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    wrench_publisher_ = get_node()->create_publisher<geometry_msgs::msg::Wrench>("wrench_topic", qos_profile2);
    std::cout << "Succesfully publishing F_ext" << std::endl;
  }

  catch (const std::exception& e) {
    fprintf(stderr,  "Exception thrown during publisher for F_ext creation at configure stage with message : %s \n",e.what());
    return CallbackReturn::ERROR;
    }
  RCLCPP_DEBUG(get_node()->get_logger(), "configured successfully");
  return CallbackReturn::SUCCESS;
}

// seems to get the initial states when the controller gets started
CallbackReturn ForceImpedanceHybridController::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  std::array<double, 16> initial_pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);  
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_pose.data()));
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  std::cout << "Completed Activation process" << std::endl;
  return CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn ForceImpedanceHybridController::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/) {                                                  
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}
//End of Callback functions.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Changes type of forces and torques from wrench to array
std::array<double, 6> ForceImpedanceHybridController::convertToStdArray(const geometry_msgs::msg::WrenchStamped& wrench) {
    std::array<double, 6> result;
    result[0] = wrench.wrench.force.x;
    result[1] = wrench.wrench.force.y;
    result[2] = wrench.wrench.force.z;
    result[3] = wrench.wrench.torque.x;
    result[4] = wrench.wrench.torque.y;
    result[5] = wrench.wrench.torque.z;
    return result;
}

//Gets the mesured external forces from the robot
void ForceImpedanceHybridController::topic_callback(const std::shared_ptr<franka_msgs::msg::FrankaRobotState> msg) {
  O_F_ext_hat_K = convertToStdArray(msg->o_f_ext_hat_k);
  arrayToMatrix(O_F_ext_hat_K, O_F_ext_hat_K_M);
}

//Updates the position (the angle) and the angular velocity of each joint
void ForceImpedanceHybridController::updateJointStates() {                                                                
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);
    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

// This is the main function of the program.
// It is the one that gets looped with 1000 Hz. 
// All other functions are either called in here or are needed at the start/end of the process.
controller_interface::return_type ForceImpedanceHybridController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {            
  
    //update all the needed data from the robot and change them to the desired format
  std::array<double, 49> mass = franka_robot_model_->getMassMatrix();
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  std::array<double, 42> jacobian_array =  franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 16> pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector); // T-Matrix (4x4)
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());
  Eigen::Matrix<double,3,3> rotation_matrix = transform.rotation();
  updateJointStates();
  
    // Initialize the message
  geometry_msgs::msg::Wrench wrench_msg;
                                      
  adapt_Sm_and_Sf(frame, F_contact_target,rotation_matrix, Sm, Sf);
  
  change_desired_position_if_force_changes(orientation, position, F_contact_target, F_request_old, position_d_, orientation_d_, position_d_target_, rotation_d_target_ /*, keepprinting*/);
  
  orientation_d_target_ = Eigen::AngleAxisd(rotation_d_target_[0], Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(rotation_d_target_[1], Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(rotation_d_target_[2], Eigen::Vector3d::UnitZ());                     //changes euler angles to quaternion
  
    //calculation of the error between the desired position and the current position
  error.head(3) << position - position_d_;
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {                                                  // fixes the issue that quaternion = - quaternion
    orientation.coeffs() << -orientation.coeffs();                                                                
  }
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);                                    // calculates the needed rotation to get to desired orientation
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail(3) << -transform.rotation() * error.tail(3);                                                         // transforms the rotational error to base frame
  
    //calculates the integrator part of the impedance control. Currently not used.
  I_error += Sm * dt * integrator_weights.cwiseProduct(error);                                                    // Integrator of PI controller for the position/orientation
  for (int i = 0; i < 6; i++){
    I_error(i,0) = std::min(std::max(-max_I(i,0),  I_error(i,0)), max_I(i,0));                                    // saturate integrater error to no get overagressive
  }

    //calculation of the impedance force
  Lambda = (jacobian * M.inverse() * jacobian.transpose()).inverse();                                             // Robot Dynamics Scipt S.79  inertia of robot
  // Theta = T*Lambda;                                                                                            // virtual scalling of the impedance (Trägheit)
  // F_impedance = -1*(Lambda * Theta.inverse() - IDENTITY) * F_ext;
    //Inertia of the robot
  switch (mode_)                                                                                                  // mode is currently set to 1, so position control
  {
  case 1:
    Theta = Lambda;
    F_impedance = -1 * (D * (jacobian * dq_) + K * error /*+ I_error*/);
    break;

  case 2:                                                                                                         // free float mode
    Theta = T*Lambda;
    F_impedance = -1*(Lambda * Theta.inverse() - IDENTITY) * F_ext;
    break;
  
  default:
    break;
  }

  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);                                                 // computes the pseudo inverse and stores it in second variable
  pseudoInverse(jacobian, jacobian_pinv);                                                           //needed for friction compensation
  calculate_tau_friction(dq_, tau_impedance, jacobian, Sm, error, jacobian_pinv); 

  get_rid_of_friction_force(jacobian_transpose_pinv, tau_friction, O_F_ext_hat_K_M, O_F_ext_hat_K_M_no_friction);
  F_ext = /*0.999 * F_ext + 0.001 */ /*O_F_ext_hat_K_M*/  O_F_ext_hat_K_M_no_friction; //Filtering       //the second part is the measured external force                                               // exponential filtering with smoothing factor = 0.1
  
    //reset Integrator if frame in which the force should be exerted gets changed while the controller is running
  if(frame != frame_before){
    I_F_error = Eigen::MatrixXd::Zero(6, 6);;
    frame_before = frame;
  }
  

    // calculates the force that should be requested.
    // ONLY FRAME 1 (BASE FRAME OF ROBOT) IS CURRENTLY WORKING
  switch (frame){
    case 1:  
      I_F_error += dt * Sf *(F_contact_des - F_ext);
      F_cmd = Sf *0.4 *(F_contact_des - F_ext) + 0.9 * I_F_error + 0.9 * F_contact_des;   // P + I + feedforward term
      break;
    case 2:
      change_wrench_to_endeffector_frame(F_ext, rotation_matrix, position);
      Sf_EE = IDENTITY;
      for (int j =0; j<6; ++j){
        if (F_contact_target(j)==0){
          Sf_EE(j,j) = 0;
        }
      }
      I_F_error += dt * Sf_EE *(F_contact_des - F_ext_EE);
      F_cmd = Sf_EE *0.4 *(F_contact_des - F_ext_EE) + 0.9 * I_F_error + 0.9 * F_contact_des;
      change_wrench_to_base_frame(F_cmd, rotation_matrix, position);
      break;
    default:
      std::cout<< "Unknown frame \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n \n";
      break;
  }

    //Calculation of the torques that the robot should exert in the next step
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                    (nullspace_stiffness_ * config_control * (q_d_nullspace_ - q_) - //bool config_control decides if we control the whole robot configuration
                    (2.0 * sqrt(nullspace_stiffness_)) * dq_);  // if config_control == false we don't care about the joint position, currently set to false.
  tau_impedance = jacobian.transpose() * Sm * (F_impedance /*+ F_repulsion + F_potential*/) + jacobian.transpose() * Sf * F_cmd;    // combine force control and impedance control
  auto tau_d_placeholder = tau_impedance + tau_nullspace + coriolis + tau_friction; //add nullspace and coriolis components to desired torque. Gets rid of part of tau_impedance that is only there because of friction
  tau_d << tau_d_placeholder;
  tau_d << saturateTorqueRate(tau_d, tau_J_d_M);  // Saturate torque rate to avoid discontinuities
  tau_J_d_M = tau_d;                              // store this value for saturateTorqueRate function in the next iteration
  
  if (outcounter % (1000/update_frequency) == 0){
    // std::cout << "F_contact_des [N]" << std::endl;
    // std::cout << F_contact_des << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "F_ext [N]" << std::endl;
    // std::cout << F_ext << std::endl;
    // std::cout << "--------" << std::endl;
    // F:cout << error << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "F_cmd [N]" << std::endl;
    // std::cout << F_cmd << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "O_F_ext_hat_K [N]" << std::endl;
    // std::cout << O_F_ext_hat_K << std::endl;
    // std::cout << "O_F_ext_hat_K_M [N]" << std::endl;
    // std::cout << O_F_ext_hat_K_M << std::endl;
    // std::cout << "norm = ";
    // std::cout << O_F_ext_hat_K_M.norm() << std::endl;
    // std::cout << "jacobian_transpose_pinv" << std::endl;
    // std::cout << jacobian_transpose_pinv << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "O_F_ext_hat_K_M_no_friction [N]" << std::endl;
    // std::cout << O_F_ext_hat_K_M_no_friction << std::endl;
    // std::cout << "Lambda  Thetha.inv(): " << std::endl;
    // std::cout << Lambda*Theta.inverse() << std::endl;
    // std::cout << "tau_d" << std::endl;
    // std::cout << tau_d << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "F_ext [N]" << std::endl;
    // std::cout << F_ext << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "norm = ";
    // std::cout << F_ext.norm() << std::endl;
    // std::cout << tau_nullspace << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "F_cmd"<< std::endl;
    // std::cout << F_cmd << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "Sm should be diagonal with Values 1"<< std::endl;
    // std::cout << Sm << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "Sf should be zero"<< std::endl;
    // std::cout << Sf << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "F_contact_target [N]"<< std::endl;
    // std::cout << F_contact_target << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << "F_contact_des [N]"<< std::endl;
    // std::cout << F_contact_des << std::endl;
    // std::cout << "--------" << std::endl;
    std::cout << "--------" << std::endl;
    // std::cout << tau_impedance << std::endl;
    // std::cout << "--------" << std::endl;
    // std::cout << coriolis << std::endl;
    // std::cout << "Inertia scaling [m]: " << std::endl;
    // std::cout << T << std::endl;
    std::cout << "Position [m]: " << std::endl;
    std::cout << position << std::endl;
    // std::cout << "Tau_d [m]: " << std::endl;
    // std::cout << tau_d << std::endl;
  }

    // sends the computed torques to the robot or the controller
  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));    
  }

    // publishes observed external force to topic
  wrench_msg.force.x = F_ext[0];
  wrench_msg.force.y = F_ext[1];
  wrench_msg.force.z = F_ext[2];
  wrench_msg.torque.x = F_ext[3];
  wrench_msg.torque.y = F_ext[4];
  wrench_msg.torque.z = F_ext[5];
  wrench_publisher_->publish(wrench_msg);
  

  outcounter++;
  update_stiffness_and_references(F_contact_des);

    //calculate the observed force
  M_dot = (M - M_old) * dt;
  M_old = M;
  update_forces(F_observed, beta_observer, gamma_observer, r_observer, K_observer, coriolis, M_dot, dq_,
                tau_d, dt, M, jacobian_transpose_pinv);

  return controller_interface::return_type::OK;
}
}

// namespace pdz_controller_library
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(pdz_controller_library::ForceImpedanceHybridController,
                       controller_interface::ControllerInterface)