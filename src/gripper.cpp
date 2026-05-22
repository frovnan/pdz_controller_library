#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <franka/gripper.h>
// If the gripper gets disconnected you have to do gripper.homing()
// Details can be found here: https://frankaemika.github.io/libfranka/0.14.1/classfranka_1_1Gripper.html#aef356f93a4c3b9d6b2532c29126d478c
//there should be some safety features added
class GripperControlNode : public rclcpp::Node {
public:
    GripperControlNode() : Node("gripper_control_node") {   // create and name the node.

        // Create a subscriber to receive gripper commands
        command_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "gripper_command", 10, std::bind(&GripperControlNode::command_callback, this, std::placeholders::_1));
    }

private:
    void command_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::cout << "Maximal width of the gripper is: " << gripper_state.max_width << std::endl;
        std::cout << "Current gripper temperature is: " << gripper_state.temperature << std::endl;
        try{
            gripper_command_ = msg->data;
            if(last_gripper_command != gripper_command_){
                std::cout<< "last_gripper_command: "<< last_gripper_command << std::endl;
                std::cout<< "gripper_command: "<< gripper_command_ << std::endl;   
                last_gripper_command = gripper_command_;

                if (gripper_command_ == "open"){
                    gripper_.move(0.08, gripper_speed);
                }
                else if (gripper_command_ == "half open"){
                    gripper_.move(0.04, gripper_speed);
                }
                else if (gripper_command_ == "close"){
                    gripper_.move(0.005, gripper_speed);
                }
                else if (gripper_command_ == "sanding block"){
                    gripper_.grasp(0.01, gripper_speed, 10, 0.08, 0.08);
                }
                else if (gripper_command_ == "hex wrench"){
                    gripper_.grasp(0.003, gripper_speed, 10, 0.003, 0.002);
                }
                else{
                    std::cout << "Error, current gripper command has no defined action" << std::endl;
                    
                }
                // If one wants to grasp something else, feel free to refer to the documentation from Franka Emika
                // https://frankaemika.github.io/libfranka/0.14.1/classfranka_1_1Gripper.html 
            }

        } catch (const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to move gripper: %s", ex.what());
        }
            
    }
    // setup stuff
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscription_;
    franka::Gripper gripper_ = franka::Gripper("192.168.1.200");
    double gripper_speed = 0.02; // [m/s]
    std::string last_gripper_command = " ";     //needs a starting value that differs from all possible commands
    std::string gripper_command_ = "open";
    
    franka::GripperState gripper_state = gripper_.readOnce();
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::cout<< "Program has started" << std::endl;
    rclcpp::spin(std::make_shared<GripperControlNode>());
    rclcpp::shutdown();
    return 0;
}
