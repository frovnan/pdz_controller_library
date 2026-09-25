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

#include "pdz_controller_library/user_input_server.hpp"
#include "pdz_controller_library/force_control_server.hpp"

#include "geometry_msgs/msg/wrench.hpp"

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/subscription.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigen>

#include <controller_interface/controller_interface.hpp>

#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "franka_hardware/franka_hardware_interface.hpp"
#include <franka_hardware/model.hpp>

#include "franka_msgs/msg/franka_robot_state.hpp"
#include "franka_msgs/msg/errors.hpp"
#include "messages_fr3/srv/set_pose.hpp"
#include "messages_fr3/srv/set_force.hpp"

#include "franka_semantic_components/franka_robot_model.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"

#define IDENTITY Eigen::MatrixXd::Identity(6, 6)

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using namespace std::chrono_literals;

namespace pdz_controller_library {

class ForceImpedanceHybridController : public controller_interface::ControllerInterface {
public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  void setPose(const std::shared_ptr<messages_fr3::srv::SetPose::Request> request,
      std::shared_ptr<messages_fr3::srv::SetPose::Response> response);
    

  ForceImpedanceHybridController();

 private:
    //Nodes
    rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr franka_state_subscriber = nullptr;      // Subscribes to the states that the robot publishes.
    rclcpp::Service<messages_fr3::srv::SetPose>::SharedPtr pose_srv_;                                           // Service to control the pose (Position and orientation)
    rclcpp::Service<messages_fr3::srv::SetForce>::SharedPtr force_srv_;                                         // Service to control the exerted forces.
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wrench_publisher_;                                 // Can be used to publish any wrench.

    //Functions
    // A quick explanation of each function can be found in cartesian_impedance_controller.cpp (or if you simply hover over the function)
    void topic_callback(const std::shared_ptr<franka_msgs::msg::FrankaRobotState> msg);
    void updateJointStates();
    //void checklimits(const Eigen::Vector3d& position, Eigen::Matrix<double, 7, 1>& tau_d);
    void update_stiffness_and_references(Eigen::Matrix<double, 6, 1>& F_contact_des);
    void arrayToMatrix(const std::array<double, 6>& inputArray, Eigen::Matrix<double, 6, 1>& resultMatrix);
    void arrayToMatrix(const std::array<double, 7>& inputArray, Eigen::Matrix<double, 7, 1>& resultMatrix);
    void adapt_Sm_and_Sf(const int& frame, const Eigen::Matrix<double, 6, 1>& F_request, const Eigen::Matrix<double,3,3>& rot_matrix, 
                        Eigen::Matrix<double, 6, 6>& Sm, Eigen::Matrix<double, 6, 6>& Sf);
    void change_desired_position_if_force_changes(const Eigen::Quaterniond orientation, const Eigen::Vector3d position, const Eigen::Matrix<double, 6, 1>& F_contact_target, 
                        Eigen::Matrix<double, 6, 1>& F_request_old, Eigen::Vector3d& position_d_, Eigen::Quaterniond& orientation_d_,
                        Eigen::Vector3d& position_d_target_, Eigen::Vector3d& rotation_d_target_);
    void calculate_tau_friction(const Eigen::Matrix<double, 7, 1>& dq_, const Eigen::Matrix<double, 7, 1>& tau_impedance,
                                                          const Eigen::Matrix<double, 6, 7>& jacobian, const Eigen::Matrix<double, 6, 6>& Sm,
                                                          const Eigen::Matrix<double, 6, 1>& error, const Eigen::MatrixXd& jacobian_pinv);
    void get_rid_of_friction_force(const Eigen::MatrixXd& jacobian_transpose_pinv, 
                                                            const Eigen::Matrix<double, 7, 1>& tau_friction, 
                                                            const Eigen::Matrix<double, 6, 1>& O_F_ext_hat_K_M,
                                                            Eigen::Matrix<double, 6, 1>& O_F_ext_hat_K_M_no_friction);
    void update_forces(Eigen::Matrix<double, 6, 1>& F_observed, 
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
                                                const Eigen::MatrixXd& jacobian_transpose_pinv);
    void change_wrench_to_base_frame(Eigen::Matrix<double, 6, 1>& F_cmd, const Eigen::Matrix<double,3,3>& rot_matrix,
                                    const Eigen::Matrix<double,3,1>& position);
    void change_wrench_to_endeffector_frame(Eigen::Matrix<double, 6, 1>& F_ext, 
                                       const Eigen::Matrix<double, 3, 3>& rot_matrix, 
                                       const Eigen::Matrix<double, 3, 1>& position);
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated, const Eigen::Matrix<double, 7, 1>& tau_J_d);  
    std::array<double, 6> convertToStdArray(const geometry_msgs::msg::WrenchStamped& wrench);
    
    //State vectors and matrices
    std::array<double, 7> q_subscribed;
    std::array<double, 7> tau_J_d = {0,0,0,0,0,0,0};
    std::array<double, 6> O_F_ext_hat_K = {0,0,0,0,0,0};
    Eigen::Matrix<double, 7, 1> q_subscribed_M;
    Eigen::Matrix<double, 7, 1> tau_J_d_M = Eigen::MatrixXd::Zero(7, 1);
    Eigen::Matrix<double, 6, 1> O_F_ext_hat_K_M = Eigen::MatrixXd::Zero(6,1);
    Eigen::Matrix<double, 7, 1> q_;
    Eigen::Matrix<double, 7, 1> dq_;
    Eigen::MatrixXd jacobian_transpose_pinv; 
    Eigen::MatrixXd jacobian_pinv;
    Eigen::Matrix<double, 7, 7> M_dot = Eigen::MatrixXd::Zero(7, 7);      
    Eigen::Matrix<double, 7, 7> M_old = Eigen::MatrixXd::Zero(7, 7);          
    

    //Robot parameters
    const int num_joints = 7;
    const std::string state_interface_name_{"robot_state"};
    const std::string robot_name_{"panda"};
    const std::string k_robot_state_interface_name{"robot_state"};
    const std::string k_robot_model_interface_name{"robot_model"};
    franka_hardware::FrankaHardwareInterface interfaceClass;
    std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
    const double delta_tau_max_{1.0};
    const double dt = 0.001;
                
    //Impedance control variables    
    Eigen::Matrix<double, 6, 7> jacobian = IDENTITY;                                         // Jacobian between Endeffector and base frame       
    Eigen::Matrix<double, 6, 6> Lambda = IDENTITY;                                           // operational space mass matrix
    Eigen::Matrix<double, 6, 6> Sm = IDENTITY;                                               // task space selection matrix for positions and rotation
    Eigen::Matrix<double, 6, 6> Sf = Eigen::MatrixXd::Zero(6, 6);                            // task space selection matrix for forces
    Eigen::Matrix<double, 6, 6> Sf_EE = Eigen::MatrixXd::Zero(6, 6);                         // endeffector space selection matrix for forces, only needed for exerting forces in the endeffector frame
    Eigen::Matrix<double, 6, 6> K =  (Eigen::MatrixXd(6,6) << 4000,   0,   0,   0,   0,   0,
                                                                0, 2000,   0,   0,   0,   0,
                                                                0,   0, 200,   0,   0,   0,  // impedance stiffness term
                                                                0,   0,   0, 130,   0,   0,
                                                                0,   0,   0,   0, 130,   0,
                                                                0,   0,   0,   0,   0,  10).finished();

    Eigen::Matrix<double, 6, 6> D =  (Eigen::MatrixXd(6,6) <<  127,   0,   0,   0,   0,   0,
                                                                0,  89,   0,   0,   0,   0,
                                                                0,   0,  28.3,   0,   0,   0,  // impedance damping term
                                                                0,   0,   0,   25,   0,   0,
                                                                0,   0,   0,   0,   25,   0,
                                                                0,   0,   0,   0,   0,   6).finished();

    // Eigen::Matrix<double, 6, 6> K =  (Eigen::MatrixXd(6,6) << 250,   0,   0,   0,   0,   0,
    //                                                             0, 250,   0,   0,   0,   0,
    //                                                             0,   0, 250,   0,   0,   0,  // impedance stiffness term
    //                                                             0,   0,   0,  80,   0,   0,
    //                                                             0,   0,   0,   0,  80,   0,
    //                                                             0,   0,   0,   0,   0,  10).finished();

    // Eigen::Matrix<double, 6, 6> D =  (Eigen::MatrixXd(6,6) <<  30,   0,   0,   0,   0,   0,
    //                                                             0,  30,   0,   0,   0,   0,
    //                                                             0,   0,  30,   0,   0,   0,  // impedance damping term
    //                                                             0,   0,   0,  18,   0,   0,
    //                                                             0,   0,   0,   0,  18,   0,
    //                                                             0,   0,   0,   0,   0,   9).finished();
    Eigen::Matrix<double, 6, 6> Theta = IDENTITY;
    Eigen::Matrix<double, 6, 6> T = (Eigen::MatrixXd(6,6) <<       1,   0,   0,   0,   0,   0,
                                                                   0,   1,   0,   0,   0,   0,
                                                                   0,   0,   1,   0,   0,   0,  // Inertia term
                                                                   0,   0,   0,   1,   0,   0,
                                                                   0,   0,   0,   0,   1,   0,
                                                                   0,   0,   0,   0,   0,   1).finished();                    // impedance inertia term

    Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;                                 // impedance damping term
    Eigen::Matrix<double, 6, 6> cartesian_damping_target_;                                   // impedance damping term
    Eigen::Matrix<double, 6, 6> cartesian_inertia_target_;                                   // impedance damping term
    Eigen::Vector3d position_d_target_ = {0.5, 0.0, 0.5};                                    // desired target position (before filtering)
    Eigen::Vector3d rotation_d_target_ = {M_PI, 0.0, 0.0};                                   // desired target orientation (before filtering) as roll, pitch, yaw
    Eigen::Quaterniond orientation_d_target_;                                                // desired target orientation (before filtering) as Quaternion
    Eigen::Vector3d position_d_;                                                             // desired position after filtering
    Eigen::Vector3d position;                                                                // actual poistion
    Eigen::Quaterniond orientation_d_;                                                       // desired orientation after filtering
    Eigen::Matrix<double, 6, 1> F_impedance;                                                 // impedance force that bring the end effector to the desired position
    Eigen::Matrix<double, 6, 1> F_contact_des = Eigen::MatrixXd::Zero(6, 1);                 // desired contact force
    Eigen::Matrix<double, 6, 1> F_contact_target = Eigen::MatrixXd::Zero(6, 1);              // desired contact force used for filtering
    Eigen::Matrix<double, 6, 1> F_ext = Eigen::MatrixXd::Zero(6, 1);                         // external forces in base frame
    Eigen::Matrix<double, 6, 1> F_ext_EE = Eigen::MatrixXd::Zero(6, 1);                      // external forces in endeffector frame
    Eigen::Matrix<double, 6, 1> F_cmd = Eigen::MatrixXd::Zero(6, 1);                         // commanded contact force
    Eigen::Matrix<double, 7, 1> q_d_nullspace_;
    Eigen::Matrix<double, 6, 1> error;                                                       // pose error (6d)
    double nullspace_stiffness_{0.001};
    double nullspace_stiffness_target_{0.001};

    // Force control variables
    int frame = 1;  //frame in which F_contact_target is. 1 = base frame; 2 = endeffector frame  FRAME 2 IS CURRENTLY NOT WORKING
    int frame_before = 1;       // frame in the last loop iteration
    Eigen::Matrix<double, 6, 1> F_request_old = Eigen::MatrixXd::Zero(6, 1);                 // requested force in the last iteration of the loop

    //Logging
    int outcounter = 0;                                                                     // counts the iteration of the update function
    const int update_frequency = 10;                                                        //frequency for outputs printed in update function

    //Integrator
    Eigen::Matrix<double, 6, 1> I_error = Eigen::MatrixXd::Zero(6, 1);                      // pose error (6d)
    Eigen::Matrix<double, 6, 1> I_F_error = Eigen::MatrixXd::Zero(6, 1);                    // force error integral
    Eigen::Matrix<double, 6, 1> integrator_weights = 
      (Eigen::MatrixXd(6,1) << 75.0, 75.0, 75.0, 75.0, 75.0, 4.0).finished();
    Eigen::Matrix<double, 6, 1> max_I = 
      (Eigen::MatrixXd(6,1) << 30.0, 30.0, 30.0, 50.0, 50.0, 2.0).finished();

   
  
    std::mutex position_and_orientation_d_target_mutex_;

    //Flags
    bool config_control = false;           // sets if we want to control the configuration of the robot in nullspace
    bool do_logging = false;               // set if we do log values

    //Filter-parameters
    double filter_params_{0.001};
    int mode_ = 1;                                                                         // mode = 1 --> cartesian impedance control (with a position) / mode = 2 --> free-float with variable inertia

    // Friction things
    const double alpha = 0.01;//constant for exponential filter in relation to static friction moment
    const Eigen::Matrix<double, 7, 1> sigma_0 = (Eigen::VectorXd(7) << 76.95, 37.94, 71.07, 44.02, 21.32, 21.83, 53).finished();
    const Eigen::Matrix<double, 7, 1> sigma_1 = (Eigen::VectorXd(7) << 0.056, 0.06, 0.064, 0.073, 0.1, 0.0755, 0.000678).finished();
    Eigen::Matrix<double, 7, 1> dz = Eigen::MatrixXd::Zero(7,1);
    Eigen::Matrix<double, 7, 1> dq_imp = Eigen::MatrixXd::Zero(7,1); //"impedance dq", dq without the nullspace-part of it
    Eigen::Matrix<double, 7, 1> z = Eigen::MatrixXd::Zero(7,1);
    Eigen::Matrix<double, 7, 1> g = (Eigen::VectorXd(7) << 1.025412896, 1.259913793, 0.8380147058, 1.005214968, 1.2928, 0.41525, 0.5341655).finished();         // those values are wrong and shouldn't be constant for joint 4,5,7 based on Viktors BA
    Eigen::Matrix<double, 7, 1> f = Eigen::MatrixXd::Zero(7,1);
    Eigen::Matrix<double, 7, 7> N = Eigen::MatrixXd::Identity(7, 7);                      // Null-space matrix, definition added by Simon on 16.05.2024
    Eigen::Matrix<double, 7, 1> dq_filtered = Eigen::MatrixXd::Zero(7,1); //rotational speed filtered for friction compensation
    Eigen::Matrix<double, 7, 1> tau_impedance_filtered = Eigen::MatrixXd::Zero(7,1); //filtered impedance torque for friction compensation
    Eigen::Matrix<double, 7, 1> tau_friction_impedance = Eigen::MatrixXd::Zero(7,1); //impedance torque needed for tau_friction
    Eigen::Matrix<double, 7, 1> tau_friction = Eigen::MatrixXd::Zero(7,1); //torque compensating friction
    Eigen::Matrix<double, 6, 6> D_friction = Eigen::DiagonalMatrix<double, 6>(40, 40, 40, 18, 18, 7); //impedance damping term for friction compensation
    Eigen::Matrix<double, 6, 6> K_friction = Eigen::DiagonalMatrix<double, 6>(300, 300, 300, 50, 50, 10); //impedance stiffness term for friction compensation
    Eigen::Matrix<double, 7, 1> offset_friction = (Eigen::VectorXd(7) << -0.05, -0.70, -0.07, -0.13, -0.1025, 0.103, -0.02).finished();
    Eigen::Matrix<double, 7, 1> beta = (Eigen::VectorXd(7) << 1.18, 0, 0.55, 0.87, 0.935, 0.54, 0.45).finished();//component b of linear friction model (a + b*dq)
    Eigen::Matrix<double, 7, 1> static_friction = (Eigen::VectorXd(7) << 1.025412896, 1.259913793, 0.8380147058, 1.005214968, 1.2928, 0.41525, 0.5341655).finished();
    Eigen::Matrix<double, 7, 1> coulomb_friction = (Eigen::VectorXd(7) << 1.025412896, 1.259913793, 0.8380147058, 0.96, 1.2928, 0.41525, 0.5341655).finished();
    Eigen::Matrix<double, 7, 1> dq_s = (Eigen::VectorXd(7) << 0, 0, 0, 0.0001, 0, 0, 0.05).finished();
    
    //torque variables
    Eigen::Matrix<double, 7, 1> tau_task = Eigen::MatrixXd::Zero(7,1);
    Eigen::Matrix<double, 7, 1> tau_nullspace = Eigen::MatrixXd::Zero(7,1);
    Eigen::Matrix<double, 7, 1> tau_d = Eigen::MatrixXd::Zero(7,1);
    Eigen::Matrix<double, 7, 1> tau_impedance = Eigen::MatrixXd::Zero(7,1);
    
    // Friction compensated force
    Eigen::Matrix<double, 6, 1> O_F_ext_hat_K_M_no_friction = Eigen::MatrixXd::Zero(6,1);


//observer parameters for force observer, taken from Bachelor Thesis of Viktor Luba: https://polybox.ethz.ch/index.php/s/iYj8ALPijKTAC2z?path=%2FFriction%20compensation
    Eigen::Matrix<double, 6, 1> F_observed = Eigen::MatrixXd::Zero(7,1);
    Eigen::Matrix<double, 7, 1> beta_observer = Eigen::MatrixXd::Zero(7,1);
    Eigen::Matrix<double, 7, 1> gamma_observer = Eigen::MatrixXd::Zero(7,1);
    Eigen::Matrix<double, 7, 7> K_observer = Eigen::MatrixXd::Identity(7,7) * 300;
    Eigen::Matrix<double, 7, 1> r_observer = Eigen::MatrixXd::Zero(7,1);
    //
  };
  

}  // namespace pdz_controller_library