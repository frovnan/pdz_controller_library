#include <pdz_controller_library/trajectory_pose_selector.hpp>

#include <rclcpp/rclcpp.hpp>
#include "messages_fr3/srv/set_pose.hpp"
#include "messages_fr3/srv/set_param.hpp"

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <array>
#include <cmath>
#include <random>
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>

// Helper function to parse expressions with pi and arbitrary fractions
double parse_pi_expression(const std::string& input) {
    std::string expr = input;
    // Remove spaces
    expr.erase(std::remove(expr.begin(), expr.end(), ' '), expr.end());
    
    // Tokenize by operators (* and /)
    std::vector<std::string> tokens;
    std::string current;
    for (char c : expr) {
        if (c == '*' || c == '/') {
            if (!current.empty()) tokens.push_back(current);
            tokens.push_back(std::string(1, c));
            current.clear();
        } else {
            current += c;
        }
    }
    if (!current.empty()) tokens.push_back(current);
    
    // Convert tokens to values and operators
    std::vector<double> values;
    std::vector<char> ops;
    
    for (size_t i = 0; i < tokens.size(); ++i) {
        if (i % 2 == 0) {  // Should be a value
            if (tokens[i] == "pi") {
                values.push_back(M_PI);
            } else {
                values.push_back(std::stod(tokens[i]));
            }
        } else {  // Should be an operator
            ops.push_back(tokens[i][0]);
        }
    }
    
    // Evaluate left-to-right (* and / have same precedence)
    double result = values[0];
    for (size_t i = 0; i < ops.size(); ++i) {
        if (ops[i] == '*') {
            result *= values[i + 1];
        } else if (ops[i] == '/') {
            result /= values[i + 1];
        }
    }
    
    return result;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("user_input_client");

    rclcpp::Client<messages_fr3::srv::SetPose>::SharedPtr pose_client =
        node->create_client<messages_fr3::srv::SetPose>("set_pose");
    auto pose_request = std::make_shared<messages_fr3::srv::SetPose::Request>();

    rclcpp::Client<messages_fr3::srv::SetParam>::SharedPtr param_client =
        node->create_client<messages_fr3::srv::SetParam>("set_param");
    auto param_request = std::make_shared<messages_fr3::srv::SetParam::Request>();

    int task_selection, pose_selection, trajectory_selection, param_selection;

    while (rclcpp::ok()){
        std::cout << "Enter the next task: \n [1] --> Change position \n [2] --> Trajectory \n [3] --> Change impedance parameters" << std::endl;
        std::cin >> task_selection;

        switch (task_selection){
            case 1:{ 
                std::cout << "Enter new goal position: \n [1] --> 0.0, 0.5, 0.5, pi, 0.0, 0.0 \n [2] --> 0.5, 0.0, 0.5, pi, 0.0, 0.0 \n [3] --> 0.5, 0.5, 0.0, pi, 0.0, 0.0 \n [4] --> Custom \n" ;
                std::cin >> pose_selection;

                switch (pose_selection){
                    case 1:{ // x = 0.0
                        pose_request->x = 0.0;
                        pose_request->y = 0.5;
                        pose_request->z = 0.5;
                        pose_request->roll = M_PI;
                        pose_request->pitch = 0.0;
                        pose_request->yaw = 0.0;
                        break;
                    }

                    case 2:{ // y = 0.0
                        pose_request->x = 0.5;
                        pose_request->y = 0.0;
                        pose_request->z = 0.5;
                        pose_request->roll = M_PI;
                        pose_request->pitch = 0.0;
                        pose_request->yaw = 0.0;
                        break;
                    }
                                
                    case 3:{ // z = 0.0
                        pose_request->x = 0.5;
                        pose_request->y = 0.5;
                        pose_request->z = 0.0;
                        pose_request->roll = M_PI;
                        pose_request->pitch = 0.0;
                        pose_request->yaw = 0.0;
                        break;
                    }

                    case 4:{ // custom input
                        std::cout << "Enter your desired position and orientation (x y z roll pitch yaw)" << std::endl;
                        std::cout << "Supported: numeric values, pi, and fractions/products (e.g., pi/2, 2*pi/3, 3/4)" << std::endl;
                        std::array<double, 6> pose;
                        std::array<std::string, 6> inputs;

                        for (size_t i = 0; i < pose.size(); ++i) {
                            std::cin >> inputs[i];
                            try {
                                pose[i] = parse_pi_expression(inputs[i]);
                            } catch (const std::exception& e) {
                                std::cerr << "Error parsing input '" << inputs[i] << "': " << e.what() << std::endl;
                                pose[i] = 0.0;
                            }
                        }

                        pose_request->x = pose[0];
                        pose_request->y = pose[1];
                        pose_request->z = pose[2];
                        pose_request->roll = pose[3];
                        pose_request->pitch = pose[4];
                        pose_request->yaw = pose[5];
                        break;
                    }
                                
                    default:{
                        pose_request->x = 0.5;
                        pose_request->y = 0.0;
                        pose_request->z = 0.4;
                        pose_request->roll = M_PI;
                        pose_request->pitch = 0.0;
                        pose_request->yaw = 0.0;
                        break;
                    }
                }
            
                auto pose_result = pose_client->async_send_request(pose_request);
                if(rclcpp::spin_until_future_complete(node, pose_result) ==  rclcpp::FutureReturnCode::SUCCESS){
                    std::cout << "Hot geklappt" << std::endl;
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Worked: %d", pose_result.get()->success);
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service setPose");
                }
                break;
            }


            case 2:{
                std::cout << "Enter new goal position: \n [1] --> Benchmark trajectory #1 (Linear motion from A --> B) \n [2] --> Benchmark trajectory #2 (Sinusoidal motion between A & B) \n [3] --> Benchmark trajectory #3 (Circle in xy-plane with z-oscillation)" << std::endl;

                std::cin >> trajectory_selection;

                double t0 = rclcpp::Clock().now().seconds();

                auto desired_pose_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("~/desired_pose", 10);
                auto trajectory_pub_ = node->create_publisher<std_msgs::msg::UInt8>("~/trajectory", 10);
                
                switch (trajectory_selection){
                    case 1:{
                        TrajectorySelector generator;
                        const double path_length = 0.5; // Length of the path in meters
                        const double tolerance = 0.05; // Tolerance for the distance between points

                        const double angle_distance = 45 * M_PI / 180.0; // Angle distance in radians
                        const double angle_tolerance = 7.5 * M_PI / 180.0; // Angle tolerance in radians

                        // Generate positions and orientations for points A, B, C, D
                        auto position_A = generator.generate_position();
                        auto position_B = generator.generate_position_within_distance(position_A, path_length, tolerance);
                        auto position_C = generator.generate_position_within_distance(position_B, path_length, tolerance);
                        auto position_D = generator.generate_position_within_distance(position_C, path_length, tolerance);

                        std::cout << "Positions A, B, C, D generated" << std::endl;

                        const int n_orientations = 200; // Number of orientations to generate for each point

                        auto orientations_A = generator.generate_orientations(n_orientations);
                        auto orientations_B = generator.generate_orientations(n_orientations);
                        auto orientations_C = generator.generate_orientations(n_orientations);
                        auto orientations_D = generator.generate_orientations(n_orientations);

                        std::cout << "Orientations A, B, C, D generated" << std::endl;
                        
                        // Initial joint configuration for the first pose (can be set to a default or previous known configuration,
                        // in this case, starting position of cartesian_impedance_controller start position in gazebo launch file)
                        Eigen::VectorXd initial_q(9);
                        initial_q << -0.016473, -0.82876, 0.00329376, -2.60491, 0.00381832, 1.76963, 0.676789, 0, 0;

                        const int number_to_keep = 15; // Number of candidates to keep after filtering based on manipulability

                        // Filter poses
                        auto filtered_poses_A = generator.filter_poses(position_A, orientations_A, number_to_keep, initial_q);
                        std::cout << "A poses filtered\n";
                        std::cout << "A size = " << filtered_poses_A.size() << "\n";
                        // Valid indices of A are all indices since we are selecting the first pose without any constraints
                        std::cout << "Creating A indices...\n";
                        std::vector<int> valid_indices_A(filtered_poses_A.size());
                        std::iota(valid_indices_A.begin(), valid_indices_A.end(), 0);
                        std::cout << "A indices created\n";
                        for (int i : valid_indices_A)
                        {
                            std::cout << "  " << i;
                        }
                        std::cout << "\n";
                        // Select random candidate from filtered poses for point A
                        std::cout << "Selecting A...\n";
                        auto pose_A_quat = generator.select_random_candidate(filtered_poses_A, valid_indices_A);
                        std::cout << "A selected\n";
                        std::cout << "A q size = " << pose_A_quat.q.size() << "\n";
                        std::cout << "A manipulability = " << pose_A_quat.manipulability << "\n";
                        std::cout << "A position = " << pose_A_quat.pose.position.transpose() << "\n";
                        // Convert quaternion of pose_A_quat to Euler angles for publishing
                        std::cout << "Converting A...\n";
                        auto pose_A_euler = generator.quaternion_to_euler(pose_A_quat.pose);
                        std::cout << "A converted" << std::endl;

                        auto filtered_poses_B = generator.filter_poses(position_B, orientations_B, number_to_keep, pose_A_quat.q);
                        std::cout << "B poses filtered\n";
                        std::cout << "B size = " << filtered_poses_B.size() << "\n";
                        // Select candidate for point B from filtered poses based on angle constraint with respect to pose_A
                        std::cout << "Selecting B...\n";
                        auto pose_B_quat = generator.select_candidate_with_angle_constraint(filtered_poses_B, pose_A_quat.pose.orientation, angle_distance, angle_tolerance);
                        std::cout << "B selected\n";
                        std::cout << "B q size = " << pose_B_quat.q.size() << "\n";
                        std::cout << "B manipulability = " << pose_B_quat.manipulability << "\n";
                        std::cout << "B position = " << pose_B_quat.pose.position.transpose() << "\n";
                        std::cout << "Converting B...\n";
                        auto pose_B_euler = generator.quaternion_to_euler(pose_B_quat.pose);
                        std::cout << "B converted" << std::endl;

                        // analogously for points C and D
                        auto filtered_poses_C = generator.filter_poses(position_C, orientations_C, number_to_keep, pose_B_quat.q);
                        std::cout << "C poses filtered\n";
                        std::cout << "C size = " << filtered_poses_C.size() << "\n";
                        std::cout << "Selecting C...\n";
                        auto pose_C_quat = generator.select_candidate_with_angle_constraint(filtered_poses_C, pose_B_quat.pose.orientation, angle_distance, angle_tolerance);
                        std::cout << "C selected\n";
                        std::cout << "C q size = " << pose_C_quat.q.size() << "\n";
                        std::cout << "C manipulability = " << pose_C_quat.manipulability << "\n";
                        std::cout << "C position = " << pose_C_quat.pose.position.transpose() << "\n";
                        std::cout << "Converting C...\n";
                        auto pose_C_euler = generator.quaternion_to_euler(pose_C_quat.pose);
                        std::cout << "C converted" << std::endl;

                        auto filtered_poses_D = generator.filter_poses(position_D, orientations_D, number_to_keep, pose_C_quat.q);
                        std::cout << "D poses filtered\n";
                        std::cout << "D size = " << filtered_poses_D.size() << "\n";
                        // Select candidate for point B from filtered poses based on angle constraint with respect to pose_A
                        std::cout << "Selecting D...\n";
                        auto pose_D_quat = generator.select_candidate_with_angle_constraint(filtered_poses_D, pose_C_quat.pose.orientation, angle_distance, angle_tolerance);
                        std::cout << "D selected\n";
                        std::cout << "D q size = " << pose_D_quat.q.size() << "\n";
                        std::cout << "D manipulability = " << pose_D_quat.manipulability << "\n";
                        std::cout << "D position = " << pose_D_quat.pose.position.transpose() << "\n";
                        std::cout << "Converting D...\n";
                        auto pose_D_euler = generator.quaternion_to_euler(pose_D_quat.pose);
                        std::cout << "D converted" << std::endl;


                        while(rclcpp::ok()) {
                            double t = rclcpp::Clock().now().seconds() - t0;
                            double T = 10.0; // Total time for the trajectory

                            if (t <= 10.0) { // Move to position A
                                pose_request->x = pose_A_euler[0];
                                pose_request->y = pose_A_euler[1];
                                pose_request->z = pose_A_euler[2];
                                pose_request->roll = pose_A_euler[3];
                                pose_request->pitch = pose_A_euler[4];
                                pose_request->yaw = pose_A_euler[5];
                            }

                            else {
                                double s = std::clamp((t - 10.0) / T, 0.0, 1.0); // Normalized time for interpolation between A and B

                                CartesianPose interpolated_pose_quat = {
                                    position_A + s * (position_B - position_A),
                                    pose_A_quat.pose.orientation.slerp(s, pose_B_quat.pose.orientation)
                                };

                                std::array<double, 6> interpolated_pose_euler = generator.quaternion_to_euler(interpolated_pose_quat);
                                
                                // Linear interpolation between position A and B over time T, 
                                // Spherical linear interpolation (slerp) between orientations A and B over time T
                                //
                                // TODO: - Discretize into ~10cm step inputs for the controller, currently continuous interpolation
                                pose_request->x = interpolated_pose_euler[0];
                                pose_request->y = interpolated_pose_euler[1];
                                pose_request->z = interpolated_pose_euler[2];
                                pose_request->roll = interpolated_pose_euler[3];
                                pose_request->pitch = interpolated_pose_euler[4];
                                pose_request->yaw = interpolated_pose_euler[5];

                                auto pose_result = pose_client->async_send_request(pose_request);

                                // --- Publish current desired pose for visualization ---
                                std_msgs::msg::Float64MultiArray desired_pose_msg;
                                desired_pose_msg.data = {   
                                    pose_request->x,
                                    pose_request->y,
                                    pose_request->z,
                                    pose_request->roll,
                                    pose_request->pitch,
                                    pose_request->yaw
                                };
                                desired_pose_pub_->publish(desired_pose_msg);

                                // --- Publish trajectory type ---
                                std_msgs::msg::UInt8 trajectory_msg;
                                trajectory_msg.data = trajectory_selection;
                                trajectory_pub_->publish(trajectory_msg);

                                if(rclcpp::spin_until_future_complete(node, pose_result) ==  rclcpp::FutureReturnCode::SUCCESS){
                                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trajectory update sent successfully.");
                                    std::cout << "Current Pose: x = " << pose_request->x << ", y = " << pose_request->y << ", z = " << pose_request->z
                                            << ", roll = " << pose_request->roll << ", pitch = " << pose_request->pitch
                                            << ", yaw = " << pose_request->yaw << std::endl;
                                } else {
                                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service setPose during trajectory.");
                                }
                            }
                        }
                        break;                        
                    }

                    case 2:{
                        const float omega = M_PI / 10.0;

                        TrajectorySelector generator;
                        const double path_length = 0.7; // Length of the path in meters
                        const double tolerance = 0.05; // Tolerance for the distance between points

                        auto position_A = generator.generate_position();
                        auto position_B = generator.generate_position_within_distance(position_A, path_length, tolerance);
                        auto position_C = generator.generate_position_within_distance(position_B, path_length, tolerance);
                        auto position_D = generator.generate_position_within_distance(position_C, path_length, tolerance);

                        while(rclcpp::ok()) {
                            double t = rclcpp::Clock().now().seconds() - t0;

                            if (t <= 5.0) { // Move to position A
                                pose_request->x = position_A[0];
                                pose_request->y = position_A[1];
                                pose_request->z = position_A[2];
                                pose_request->roll = position_A[3];
                                pose_request->pitch = position_A[4];
                                pose_request->yaw = position_A[5];
                            }

                            pose_request->x = (position_A[0] + position_B[0]) / 2.0 + (position_A[0] - position_B[0]) / 2.0 * cos(omega * (t - 5.0));
                            pose_request->y = (position_A[1] + position_B[1]) / 2.0 + (position_A[1] - position_B[1]) / 2.0 * cos(omega * (t - 5.0));
                            pose_request->z = (position_A[2] + position_B[2]) / 2.0 + (position_A[2] - position_B[2]) / 2.0 * cos(omega * (t - 5.0));
                            pose_request->roll = M_PI;
                            pose_request->pitch = 0.0;
                            pose_request->yaw = 0.0;

                            auto pose_result = pose_client->async_send_request(pose_request);

                            // --- Publish current desired pose for visualization ---
                            std_msgs::msg::Float64MultiArray desired_pose_msg;
                            desired_pose_msg.data = {   
                                pose_request->x,
                                pose_request->y,
                                pose_request->z,
                                pose_request->roll,
                                pose_request->pitch,
                                pose_request->yaw
                            };
                            desired_pose_pub_->publish(desired_pose_msg);

                            // --- Publish trajectory type ---
                            std_msgs::msg::UInt8 trajectory_msg;
                            trajectory_msg.data = trajectory_selection;
                            trajectory_pub_->publish(trajectory_msg);

                            if(rclcpp::spin_until_future_complete(node, pose_result) ==  rclcpp::FutureReturnCode::SUCCESS){
                                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trajectory update sent successfully.");
                                std::cout << "Current Pose: x = " << pose_request->x << ", y = " << pose_request->y << ", z = " << pose_request->z
                                        << ", roll = " << pose_request->roll << ", pitch = " << pose_request->pitch
                                        << ", yaw = " << pose_request->yaw << std::endl;
                            } else {
                                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service setPose during trajectory.");
                            }
                        }
                        break;
                    }

                    case 3:{
                        const float amplitude_xy = 0.2; // Amplitude in xy
                        const float amplitude_z = 0.1; // Amplitude in z
                        const float omega = M_PI / 6.0; // Frequency of the trajectory

                        while(rclcpp::ok()) {
                            double t = rclcpp::Clock().now().seconds() - t0;

                            pose_request->x = 0.4 + amplitude_xy * sin(omega * t);
                            pose_request->y = amplitude_xy * cos(omega * t);
                            pose_request->z = 0.5 + amplitude_z * sin(1.5 * omega * t);
                            pose_request->roll = M_PI;
                            pose_request->pitch = 0.0;
                            pose_request->yaw = 0.0;

                            auto pose_result = pose_client->async_send_request(pose_request);

                            // --- Publish current desired pose for visualization ---
                            std_msgs::msg::Float64MultiArray desired_pose_msg;
                            desired_pose_msg.data = {   
                                pose_request->x,
                                pose_request->y,
                                pose_request->z,
                                pose_request->roll,
                                pose_request->pitch,
                                pose_request->yaw
                            };
                            desired_pose_pub_->publish(desired_pose_msg);

                            // --- Publish trajectory type ---
                            std_msgs::msg::UInt8 trajectory_msg;
                            trajectory_msg.data = trajectory_selection;
                            trajectory_pub_->publish(trajectory_msg);

                            if(rclcpp::spin_until_future_complete(node, pose_result) ==  rclcpp::FutureReturnCode::SUCCESS){
                                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trajectory update sent successfully.");
                                std::cout << "Current Pose: x = " << pose_request->x << ", y = " << pose_request->y << ", z = " << pose_request->z
                                        << ", roll = " << pose_request->roll << ", pitch = " << pose_request->pitch
                                        << ", yaw = " << pose_request->yaw << std::endl;
                            } else {
                                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service setPose during trajectory.");
                            }
                        }
                        break;
                    }
                    default:{
                        pose_request->x = 0.5;
                        pose_request->y = 0.0;
                        pose_request->z = 0.4;
                        pose_request->roll = M_PI;
                        pose_request->pitch = 0.0;
                        pose_request->yaw = 0.0;
                        break;
                    }
                }
                break;
            }

            case 3:{                                
                std::cout << "Enter new inertia: \n [1] --> N/A \n [2] --> N/A \n [3] --> N/A\n";
                std::cin >> param_selection;

                switch(param_selection){
                    case 1:{
                        param_request->a = 2;
                        param_request->b = 0.5;
                        param_request->c = 0.5;
                        param_request->d = 2;
                        param_request->e = 0.5;
                        param_request->f = 0.5;
                        break;
                    }

                    default:{
                        param_request->a = 1;
                        param_request->b = 1;
                        param_request->c = 1;
                        param_request->d = 1;
                        param_request->e = 1;
                        param_request->f = 1;
                        break;
                    }
                }

                auto param_result = param_client->async_send_request(param_request);
                if(rclcpp::spin_until_future_complete(node, param_result) ==  rclcpp::FutureReturnCode::SUCCESS){
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %d", param_result.get()->success);
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service setParam");
                }
                break;
            }

            default:{
                std::cout << "Invalid selection, please try again\n";
                break;   
            }
        }
    }    
    
    rclcpp::shutdown();
    return 0;
}