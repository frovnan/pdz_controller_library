#include <rclcpp/rclcpp.hpp>
#include "messages_fr3/srv/set_pose.hpp"
#include "messages_fr3/srv/set_param.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <array>
#include <cmath>

#include <string>
#include <vector>
#include <algorithm>

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

    int task_selection, pose_selection, param_selection;

    while (rclcpp::ok()){
        std::cout << "Enter the next task: \n [1] --> Change position \n [2] --> Change impedance parameters" << std::endl;
        std::cin >> task_selection;

        switch (task_selection){
            case 1:{ 
                std::cout << "Enter new goal position: \n [1] --> 0.0, 0.5, 0.5, pi, 0.0, pi/2 \n [2] --> 0.5, 0.0, 0.5, pi, 0.0, pi/2 \n [3] --> 0.5, 0.5, 0.0, pi, 0.0, pi/2 \n [4] --> Custom \n" ;
                std::cin >> pose_selection;

                switch (pose_selection){
                    case 1:{ // x = 0.0
                        pose_request->x = 0.0;
                        pose_request->y = 0.5;
                        pose_request->z = 0.5;
                        pose_request->roll = M_PI;
                        pose_request->pitch = 0.0;
                        pose_request->yaw = M_PI_2;
                        break;
                    }

                    case 2:{ // y = 0.0
                        pose_request->x = 0.5;
                        pose_request->y = 0.0;
                        pose_request->z = 0.5;
                        pose_request->roll = M_PI;
                        pose_request->pitch = 0.0;
                        pose_request->yaw = M_PI_2;
                        break;
                    }
                                
                    case 3:{ // z = 0.0
                        pose_request->x = 0.5;
                        pose_request->y = 0.5;
                        pose_request->z = 0.0;
                        pose_request->roll = M_PI;
                        pose_request->pitch = 0.0;
                        pose_request->yaw = M_PI_2;
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
                        pose_request->yaw = M_PI_2;
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