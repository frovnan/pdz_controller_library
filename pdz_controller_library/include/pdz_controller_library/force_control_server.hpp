#pragma once                            //make sure this file is only included once by compiler, if done multiple times this may cause a compiler error

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <messages_fr3/srv/set_force.hpp>            // Points to an automatic generated file in the include folder

namespace pdz_controller_library {

class UserInputForceServer {
public:
  UserInputForceServer (Eigen::Matrix<double, 6, 1>* force_and_moment,int* inputframe) :    //constructor
  F_contact_target(force_and_moment), frame(inputframe){}
    int main(int argc, char **argv);

private:
  Eigen::Matrix<double, 6, 1>* F_contact_target;
  int* frame;
  void setForce(const std::shared_ptr<messages_fr3::srv::SetForce::Request> request, 
    std::shared_ptr<messages_fr3::srv::SetForce::Response> response);
};
}

