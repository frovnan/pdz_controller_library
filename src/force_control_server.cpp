#include <chrono>
#include <iostream>
#include <cstdlib>
#include <memory>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "pdz_controller_library/force_control_server.hpp"
#include "pdz_controller_library/cartesian_impedance_controller.hpp"
namespace pdz_controller_library {

void UserInputForceServer::setForce(const std::shared_ptr<messages_fr3::srv::SetForce::Request> request, 
    std::shared_ptr<messages_fr3::srv::SetForce::Response> /*response*/)
{
    (*F_contact_target)[0] = request->x_force;
    (*F_contact_target)[1] = request->y_force;
    (*F_contact_target)[2] = request->z_force;
    (*F_contact_target)[3] = request->x_torque;
    (*F_contact_target)[4] = request->y_torque;
    (*F_contact_target)[5] = request->z_torque;
    (*frame) = request->frame;

}

int UserInputForceServer::main(int /*argc*/, char** /***argv*/) {    
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("force_control_service");
    rclcpp::Service<messages_fr3::srv::SetForce>::SharedPtr pose_service = node->create_service<messages_fr3::srv::SetForce> // creates ROS2 service (resp. server) with name set_force
        ("set_force", std::bind(&UserInputForceServer::setForce, this, std::placeholders::_1, std::placeholders::_2));
    

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to exert forces.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 
}