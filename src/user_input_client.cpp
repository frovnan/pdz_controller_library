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

                        // Initial joint configuration for the first pose (can be set to a default or previous known configuration,
                        // in this case, starting position of cartesian_impedance_controller start position in gazebo launch file
                        Eigen::VectorXd initial_q(9);
                        initial_q << -0.016473, -0.82876, 0.00329376, -2.60491, 0.00381832, 1.76963, 0.676789, 0, 0;

                        int number_of_points = 4;

                        std::vector<CartesianPose> poses = generator.generate_poses(initial_q, number_of_points);
                        
                        const double hold_time = 5.0;
                        const double move_time = 10.0;

                        while (rclcpp::ok()) {
                            double t = rclcpp::Clock().now().seconds() - t0;

                            // TODO: include time slot for moving to pose A

                            // Determine which segment we are currently in
                            double segment_duration = hold_time + move_time;

                            size_t n = static_cast<int>(t / segment_duration);

                            // Stop once the final pose has been reached
                            if (n >= poses.size() - 1) {break;}

                            // Time elapsed within the current segment
                            double t_segment = std::fmod(t, segment_duration);

                            const CartesianPose& start = poses[n];
                            const CartesianPose& end = poses[n + 1];

                            CartesianPose interpolated_pose;

                            if (t_segment < hold_time) {
                                // Hold the starting pose
                                interpolated_pose = start;
                            } else {
                                // Move from start to end
                                double s = std::clamp((t_segment - hold_time) / move_time, 0.0, 1.0);

                                interpolated_pose = {
                                    start.position + s * (end.position - start.position),
                                    start.orientation.slerp(s, end.orientation)
                                };
                            }

                            // Convert to Euler angles only for publishing
                            std::array<double, 6> pose_euler = generator.quaternion_to_euler(interpolated_pose);

                            // TODO: Discretize into ~10 cm step inputs for the controller

                            pose_request->x = pose_euler[0];
                            pose_request->y = pose_euler[1];
                            pose_request->z = pose_euler[2];
                            pose_request->roll = pose_euler[3];
                            pose_request->pitch = pose_euler[4];
                            pose_request->yaw = pose_euler[5];

                            auto pose_result = pose_client->async_send_request(pose_request);

                            // --- Publish current desired pose for logging ---
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
                            /*
                            if(rclcpp::spin_until_future_complete(node, pose_result) ==  rclcpp::FutureReturnCode::SUCCESS){
                                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trajectory update sent successfully.");
                                std::cout << "Current Pose: x = " << pose_request->x << ", y = " << pose_request->y << ", z = " << pose_request->z
                                        << ", roll = " << pose_request->roll << ", pitch = " << pose_request->pitch
                                        << ", yaw = " << pose_request->yaw << std::endl;
                            } else {
                                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service setPose during trajectory.");
                            }
                            */
                        }                      
                                                        
                        // --- Publish trajectory type ---
                        std_msgs::msg::UInt8 trajectory_msg;
                        trajectory_msg.data = trajectory_selection;
                        trajectory_pub_->publish(trajectory_msg);
                    }
                        break;                        
                    
                    /*
                    case 2:{
                        const float omega = M_PI / 10.0;

                        TrajectorySelector generator;
                        const double path_length = 0.7; // Length of the path in meters
                        const double tolerance = 0.05; // Tolerance for the distance between points

                        auto position_A = generator.generate_position();
                        auto position_B = generator.generate_position_with_constraints(position_A, path_length, tolerance);
                        auto position_C = generator.generate_position_with_constraints(position_B, path_length, tolerance);
                        auto position_D = generator.generate_position_with_constraints(position_C, path_length, tolerance);

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
                    */
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