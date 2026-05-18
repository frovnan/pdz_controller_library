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

#pragma once

#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <unistd.h>
#include <thread>
#include <chrono>        
#include <future> 

#include "pdz_controller_library/user_input_server.hpp"

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/subscription.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigen>

#include <controller_interface/controller_interface.hpp>

#include "franka_msgs/msg/franka_robot_state.hpp"
#include "franka_msgs/msg/errors.hpp"
#include "messages_fr3/srv/set_pose.hpp"
#include <std_msgs/msg/string.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#define IDENTITY Eigen::MatrixXd::Identity(6, 6)

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using Vector7d = Eigen::Matrix<double, 7, 1>;

namespace pdz_controller_library {

/**
 * joint impedance example controller get desired pose and use inverse kinematics LMA
 * (Levenberg-Marquardt) from Orocos KDL. IK returns the desired joint positions from the desired
 * pose. Desired joint positions are fed to the impedance control law together with the current
 * joint velocities to calculate the desired joint torques.
 */
class JointImpedanceIkController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  void setPose(const std::shared_ptr<messages_fr3::srv::SetPose::Request> request, 
              std::shared_ptr<messages_fr3::srv::SetPose::Response> response);

  void calculate_tau_friction();

 private:
  //Nodes
  //robot description
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr robot_description_sub_;
  std::promise<std::string> robot_description_promise_;
  rclcpp::Service<messages_fr3::srv::SetPose>::SharedPtr pose_srv_;
  const int num_joints_ = 7;
  const int num_fingers_ = 2;
  const std::string robot_name_{"fr3"};
  const std::string state_interface_name_{"robot_state"};

  //Functions
  void update_joint_states();
  void reference_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void update_stiffness_and_references();
  int counter_ = 0;

  //subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr desired_pose_sub_;

  /**
   * @brief Calculates the new pose based on the initial pose.
   *
   * @return  Eigen::Vector3d calculated sinosuidal period for the x,z position of the pose.
   */
  Eigen::Vector3d compute_new_position();


  /**
   * @brief computes the torque commands based on impedance control law with compensated coriolis
   * terms
   *
   * @return Eigen::Vector7d torque for each joint of the robot
   */
  Vector7d compute_torque_command(const Vector7d& joint_positions_desired,
                                  const Vector7d& joint_positions_current,
                                  const Vector7d& joint_velocities_current);

  /**
   * @brief assigns the Kp, Kd and arm_id parameters
   *
   * @return true when parameters are present, false when parameters are not available
   */
  bool assign_parameters();

  int control_mode = 0; // 0 for normal, 1 for free float (all control_mode can be removed)
  
  
  //compliance
  Eigen::Matrix<double, 6, 6> K_cartesian_desired =  (Eigen::MatrixXd(6,6) << 2000,   0,   0,   0,   0,   0,
                                                                              0, 2000,   0,   0,   0,   0,
                                                                              0,   0, 50,   0,   0,   0,  // impedance stiffness term
                                                                              0,   0,   0, 70,   0,   0,
                                                                              0,   0,   0,   0, 70,   0,
                                                                              0,   0,   0,   0,   0,  20).finished();

  Eigen::Matrix<double, 6, 6> D_cartesian_desired =  (Eigen::MatrixXd(6,6) <<  100,   0,   0,   0,   0,   0,
                                                                                0,  100,   0,   0,   0,   0,
                                                                                0,   0,  16,   0,   0,   0,  // impedance damping term
                                                                                0,   0,   0,   20,   0,   0,
                                                                                0,   0,   0,   0,   20,   0,
                                                                                0,   0,   0,   0,   0,   9).finished();


  Eigen::Quaterniond orientation_;
  Eigen::Vector3d position_;
  

  double trajectory_period_{0.001};
  const bool k_elbow_activated_{false};
  bool initialization_flag_{true};

  std::string arm_id_{"fr3"};

  double elapsed_time_{0.0};
  pinocchio::Model model_;
  pinocchio::Data data_;
  int end_effector_frame_id_; // Frame ID for the end-effector

  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};
  
  Eigen::VectorXd q_;
  Eigen::VectorXd dq_;
  Vector7d dq_filtered;
  Eigen::VectorXd k_gains_;
  Eigen::VectorXd d_gains_;

// jacobian and nullspaces
  Eigen::MatrixXd jacobian; //Jacobian of the end-effector
  Eigen::MatrixXd jacobian_pinv; //pseudoinverse jacobian matrix
  Eigen::MatrixXd jacobian_transpose_pinv; //pseudoinverse of the transposed jacobian matrix
  Eigen::Matrix<double, 7, 7> N = Eigen::MatrixXd::Zero(7, 7); //Nullspace matrix of the end-effector
  double nullspace_stiffness_{0.001};
  double nullspace_stiffness_target_{0.001};
  

  Eigen::Vector3d position_d_target_ = {0.5, 0.0, 0.5};
  Eigen::Vector3d rotation_d_target_ = {0.0, 0.0, 0.0};
  Eigen::Quaterniond orientation_d_target_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_; 
  Eigen::Vector3d position_model;
  Eigen::Quaterniond orientation_model;

  std::vector<double> joint_positions_desired_;
  std::vector<double> joint_positions_current_{0, 0, 0, 0, 0, 0, 0};
  std::vector<double> joint_velocities_current_{0, 0, 0, 0, 0, 0, 0};
  std::vector<double> joint_efforts_current_{0, 0, 0, 0, 0, 0, 0};
  
  Eigen::VectorXd q_des_ik_;

  Vector7d joint_positions_desired_eigen;
  Vector7d joint_positions_current_eigen;
  Vector7d joint_velocities_current_eigen;

  // IK parameters
  double kp_pos = 2.0; // proportional gain for position error
  double kp_ori = 1.0; // proportional gain for orientation error
  double alpha = 1.0; // step size for the update of the desired joint positions from the ik solution
  const double dt = 0.001; // 1 kHz control loop

  //Filter-parameters
  double filter_params_{0.01};
};
}  // namespace pdz_controller_library
