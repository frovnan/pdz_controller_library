#include <rclcpp/rclcpp.hpp>
#include "messages_fr3/srv/set_pose.hpp"
#include "messages_fr3/srv/set_param.hpp"
#include "messages_fr3/srv/set_force.hpp"
#include "std_msgs/msg/string.hpp"

#include <string>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <array>
#include <cmath>

//publish the requested pose
void publish_pose(const rclcpp::Client<messages_fr3::srv::SetPose>::SharedPtr pose_client, const std::shared_ptr<messages_fr3::srv::SetPose::Request> pose_request, const std::shared_ptr<rclcpp::Node> node){
    auto pose_result = pose_client->async_send_request(pose_request);
        if(rclcpp::spin_until_future_complete(node, pose_result) ==  rclcpp::FutureReturnCode::SUCCESS){
            std::cout << "Hot geklappt\n";
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Worked: %d", pose_result.get()->success);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service setPose");
        }
}

//publish the requested force
void publish_force(const rclcpp::Client<messages_fr3::srv::SetForce>::SharedPtr force_client, const std::shared_ptr<messages_fr3::srv::SetForce::Request> force_request, const std::shared_ptr<rclcpp::Node> node){
    auto force_result = force_client->async_send_request(force_request);
        if(rclcpp::spin_until_future_complete(node, force_result) ==  rclcpp::FutureReturnCode::SUCCESS){
            std::cout << "Hot geklappt\n";
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Worked: %d", force_result.get()->success);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service setForce");
        }
}
// publish the requested gripper position
void publish_gripper_task(const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher,
                  const std::string& message_content, 
                  const std::shared_ptr<rclcpp::Node> node){ 
    // Create a String message
    auto message = std::make_shared<std_msgs::msg::String>();
    message->data = message_content;  // Set the content of the message

    // Publish the message
    publisher->publish(*message);

    RCLCPP_INFO(node->get_logger(), "Published message: %s", message->data.c_str());

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("user_input_client");
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_command_publisher_;

    //Create the client for set_pose
    rclcpp::Client<messages_fr3::srv::SetPose>::SharedPtr pose_client =
        node->create_client<messages_fr3::srv::SetPose>("set_pose");
    auto pose_request = std::make_shared<messages_fr3::srv::SetPose::Request>();
    //std::shared_ptr<messages_fr3::srv::SetPose::Request> pose_request

    //Create the client for set_param
    rclcpp::Client<messages_fr3::srv::SetParam>::SharedPtr param_client =
        node->create_client<messages_fr3::srv::SetParam>("set_param");
    auto param_request = std::make_shared<messages_fr3::srv::SetParam::Request>();

    
    //Create the client for set_force
    // COMMENT THIS IF YOU USE an extra terminal with:
    //          ros2 run cartesian_impedance_control force_control_client
    rclcpp::Client<messages_fr3::srv::SetForce>::SharedPtr force_client =
        node->create_client<messages_fr3::srv::SetForce>("set_force");
    auto force_request = std::make_shared<messages_fr3::srv::SetForce::Request>();
    
    //establish contact with the topic gripper_command
    try {
        rclcpp::QoS qos_profile3(1); // Depth of the message queue aka does only keep one message all older ones get deleted
        qos_profile3.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        gripper_command_publisher_ = node->create_publisher<std_msgs::msg::String>("gripper_command", qos_profile3);
        std::cout << "Succesfully publishing gripper commands" << std::endl;
    }

    catch (const std::exception& e) {
        fprintf(stderr,  "Exception thrown during publisher for gripper command creation at configure stage with message : %s \n",e.what());
        }
    
    // initialize all the variables
    int task_selection, param_selection;
    std::string pose_selection_str;
    std::string gripper_command;
    char pose_selection_char;
    double store;

    // initialize variables for the sanding task
    double amplitude = 0.07;            //won't reach the maximal amplitude due to filter parameter filter_params in cartesian_impedance_controller.cpp
    double frequency = 1.0;
    
    //initialize variables to allow relative steering from the start on
    // This could be made better by getting the current position of the robot.
    pose_request->x = 0.5;
    pose_request->y = 0.0;
    pose_request->z = 0.4;
    pose_request->roll = M_PI;
    pose_request->pitch = 0.0;
    pose_request->yaw = M_PI_2;

    while (rclcpp::ok()){
        // TO BE ABLE TO CHANGE THE IMPEDANCE PARAMETERS ONLY DELETE THE COMMENTS OF STYLE /**/
        /*                 // uncommented because I don't need the free float mode
        // IF THE PART THAT IS COMMENTED IS NEEDED AGAIN MOVE ALL CODE THAT IS CURRENTLY ACTIVE TWO TABS TO THE RIGHT
        std::cout << "Enter the next task: \n [1] --> Change position \n [2] --> Change impedance parameters" << std::endl;
        std:: cin >> task_selection;
        switch (task_selection){
            case 1:{ */
        //publish_pose(pose_client,pose_request,node);
        std::cout << "Enter new goal position: \n [1] --> 0.5, -0.4, 0.5 \n [2] --> DO NOT USE \n [3] --> starting pose of colaboration routine but random pose\n [4] --> 0.298, 0.7, 0.1 \n [5] --> 0.6, 0.0, 0.07 \n ";
        std::cout << "[6] --> Do the collaboration routine \n [7] --> START POS FOR SCREW \n [j] --> Reduce current z-Position by 1 cm \n [u] --> Increase current z-Position by 1 cm \n [w] --> Reduce current x-Position by 0.1 cm \n ";
        std::cout << "[a] --> Reduce current y-Position by 0.1 cm \n [s] --> Increase current x-Position by 0.1 cm \n [d] --> Increase current y-Position by 0.1 cm \n";
        std::cout << "[l] --> Reduce current z-Position by 0.1 cm \n [o] --> Increase current z-Position by 0.1 cm \n ";
        std::cout << "All turning commands bellow are in the robots base frame marked on the robot. Positve and negative are according to right hand rule.\n ";
        std::cout << "[v] --> Turn -90 deg around x-axis.\n [b] --> Turn +90 deg around x-axis\n ";
        std::cout << "[t] --> Turn ~ -10 deg around y-axis.\n [g] --> Turn ~ +10 deg around y-axis.\n ";
        std::cout << "[y] --> Turn -90 deg around z-axis.\n [x] --> Turn +90 deg around z-axis\n";
        std::cout << "[0] --> Sanding mode: Oszillate around a given position\n";
        std::cout << "[,] --> Close the gripper.\n";
        std::cout << "[.] --> Release object. Open the gripper.\n";
        std::cout << "[-] --> Do the screw routine\n";
        std::cout << "[8] --> Grasp sanding block\n";
        std::cout << "[9] --> Grasp hex wrench\n";
        std::cin >> pose_selection_str;
        
        // make sure you get a one letter input only
        if (pose_selection_str.length() != 1) {
            std::cout << "Invalid input. Please enter a valid input." << std::endl;
            continue;
        }
        pose_selection_char = pose_selection_str[0];

        switch (pose_selection_char){
            case '1':{
                pose_request->x = 0.5;
                pose_request->y = -0.4;
                pose_request->z = 0.5;
                pose_request->roll = M_PI;
                pose_request->pitch = 0.0;
                pose_request->yaw = -M_PI_2;
                break;
            }
            case '2':{
                std::cout << "Enter your desired position and orientation" << std::endl;
                std::array<double, 6> pose;
                for (long unsigned int i = 0; i<pose.size(); ++i){
                    std::cin >> pose[i];
                }
                pose_request->x = pose[0];
                pose_request->y = pose[1];
                pose_request->z = pose[2];
                pose_request->roll = pose[3];
                pose_request->pitch = pose[4];
                pose_request->yaw = pose[5];
                break;
            }
            case '3':{
                pose_request->x = 0.4;
                pose_request->y = 0.3;
                pose_request->z = 0.3;
                pose_request->roll = M_PI-0.5;
                pose_request->pitch = 0.3;
                pose_request->yaw = -M_PI_2-0.2;
                break;
            }
            case '4':{
                pose_request->x = 0.298;
                pose_request->y = 0.7;
                pose_request->z = 0.1;
                pose_request->roll = M_PI;
                pose_request->pitch = 0.0;
                pose_request->yaw = -M_PI_2;
                break;
            }
            case '5':{
                pose_request->x = 0.6;
                pose_request->y = 0.0;
                pose_request->z = 0.065;
                pose_request->roll = M_PI - 0.3;
                pose_request->pitch = 0.0;
                pose_request->yaw = -M_PI_2;
                break;
            }
            case '6':{
                std::cout << "Collaboration routine started. The robot will wait for 5 seconds.\n";
                auto start_time = std::chrono::steady_clock::now();
                auto elapsedTime = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 10){
                }       //wait 5 seconds
                pose_request->x = 0.6;
                pose_request->y = 0.2;
                pose_request->z = 0.11;
                pose_request->roll = M_PI-0.4;
                pose_request->pitch = 0.0;
                pose_request->yaw = -M_PI_2;
                std::cout << "The robot will now move near the target object. He has currently 5 seconds to do so.\n";
                publish_pose(pose_client,pose_request,node);
                start_time = std::chrono::steady_clock::now();
                elapsedTime = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count()  < 7.0){
                }

                //do force shit 
                force_request->x_force = 0.0;
                force_request->y_force = 0.0;
                force_request->z_force = -3.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                force_request->frame = 1;
                std::cout << "We exert a force of -3N in z-direction to create a contact.\n";
                publish_force(force_client, force_request, node);
                start_time = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 3.0){
                }
                force_request->x_force = 0.0;
                force_request->y_force = 0.0;
                force_request->z_force = -7.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                force_request->frame = 1;
                std::cout << "We should now be in contact. The force gets increased to 7N.\n";
                publish_force(force_client, force_request, node);
                double start_pos_x = pose_request->x;
                elapsedTime = std::chrono::steady_clock::now();
                start_time = std::chrono::steady_clock::now();
                std::cout << "The robot starts the 10 seconds of cleaning.\n";
                while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 10) {
                    // Calculate elapsed time in seconds
                    auto elapsedTime = std::chrono::steady_clock::now() - start_time;
                    double elapsedSeconds = std::chrono::duration_cast<std::chrono::duration<double>>(elapsedTime).count();

                    // Update pose_request using the elapsed time
                    pose_request->x = start_pos_x - 0.15/10*elapsedSeconds;
                    pose_request->y = pose_request->y;
                    pose_request->z = pose_request->z;
                    pose_request->roll = pose_request->roll;
                    pose_request->pitch = pose_request->pitch;
                    pose_request->yaw = pose_request->yaw;

                    // Publish the pose
                    publish_pose(pose_client, pose_request, node);

                    // Print the Y value and the elapsed time
                    //std::cout << "pose_request-->y: " << pose_request->y << std::endl;
                    std::cout << "Elapsed time (seconds): " << elapsedSeconds << std::endl;
                }
                force_request->x_force = 0.0;
                force_request->y_force = 0.0;
                force_request->z_force = 0.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                publish_force(force_client, force_request, node);
                std::cout << "Force control deactivated, only impedance control now.\n";
                pose_request->x = 0.45;
                pose_request->y = 0.2;
                pose_request->z = 0.3;
                pose_request->roll = M_PI;
                pose_request->pitch = 0.0;
                pose_request->yaw = -M_PI_2;
                std::cout << "The robot will now move near the target object. He has currently 5 seconds to do so.\n";
                publish_pose(pose_client,pose_request,node);
                start_time = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 3.0){
                }
                break;
            }
            case '7':{
                pose_request->x = 0.45;
                pose_request->y = 0.298;
                pose_request->z = 0.176;
                pose_request->roll = M_PI ;
                pose_request->pitch = 0.0;
                pose_request->yaw = -M_PI_2;
                break;
            }
            case 'j':{
                store = pose_request->z;
                pose_request->z = store - 0.01;
                break;
            }
            case 'u':{
                store = pose_request->z;
                pose_request->z = store + 0.01;
                break;
            }
            case 'w':{
                store = pose_request->x;
                pose_request->x = store - 0.05;
                break;
            }
            case 'a':{
                store = pose_request->y;
                pose_request->y = store - 0.05;
                break;
            }
            case 's':{
                store = pose_request->x;
                pose_request->x = store + 0.05;
                break;
            }
            case 'd':{
                store = pose_request->y;
                pose_request->y = store + 0.05;
                break;
            }
            case 'l':{
                store = pose_request->z;
                pose_request->z = store - 0.05;
                break;
            }
            case 'o':{
                store = pose_request->z;
                pose_request->z = store + 0.05;
                break;
            }
            case 'x':{
                store = pose_request->yaw;
                if(store >= 0){
                    pose_request->yaw = store - M_PI_2;
                }
                else{
                    std::cout << "Nah, robots hardware limits say no.\n";
                }
                break;
            }
            case 'y':{
                store = pose_request->yaw;
                if(store <= 0){
                    pose_request->yaw = store + M_PI_2;
                }
                else{
                    std::cout << "Nah, robots hardware limits say no.\n";
                }
                break;
            }
            case 'v':{
                store = pose_request->roll;
                if(store >= M_PI){
                    pose_request->roll = store - M_PI_2;
                }
                else{
                    std::cout << "Nah, robots hardware limits say no.\n";
                }
                break;
            }
            case 'b':{
                store = pose_request->roll;
                if(store <= M_PI){
                    pose_request->roll = store + M_PI_2;
                }
                else{
                    std::cout << "Nah, robots hardware limits say no.\n";
                }
                break;
            }
            case 't':{
                store = pose_request->pitch;
                if(store <= (M_PI_2-0.2)){
                    pose_request->pitch = store + 0.2;
                }
                else{
                    std::cout << "Nah, robots hardware limits say no.\n";
                }
                break;
            }
            case 'g':{
                store = pose_request->pitch;
                if(store >= 0.2){
                    pose_request->pitch = store - 0.2;
                }
                else{
                    std::cout << "Nah, robots hardware limits say no.\n";
                }
                break;
            }
            case '0':{                              //moves the robot in y-direction in a sinusoidal path to enable sanding
                /*
                std::cout << "Please enter the desired amplitude [m]\n";
                std::cin >> amplitude;
                std::cout << "Please enter the desired frequency [Hz]\n";
                std::cin >> frequency;
                */
               std::cout << "Sanding mode started. The robot will wait for 5 seconds.\n";
                auto start_time = std::chrono::steady_clock::now();
                auto elapsedTime = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 5){
                }       //wait 5 seconds
                pose_request->x = 0.6;
                pose_request->y = 0.0;
                pose_request->z = 0.065;
                pose_request->roll = M_PI - 0.3;
                pose_request->pitch = 0.0;
                pose_request->yaw = -M_PI_2;
                std::cout << "The robot will now move near the target object. He has currently 5 seconds to do so.\n";
                publish_pose(pose_client,pose_request,node);
                start_time = std::chrono::steady_clock::now();
                elapsedTime = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count()  < 7.0){
                }

                //do force shit 
                force_request->x_force = 0.0;
                force_request->y_force = 0.0;
                force_request->z_force = -3.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                std::cout << "We exert a force of -3N in z-direction to create a contact.\n";
                publish_force(force_client, force_request, node);
                start_time = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 5.0){
                }
                force_request->x_force = 0.0;
                force_request->y_force = 0.0;
                force_request->z_force = -8.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                std::cout << "We should now be in contact. The force gets increased to 12N.\n";
                publish_force(force_client, force_request, node);
                double start_pos_x = pose_request->x;
                elapsedTime = std::chrono::steady_clock::now();
                start_time = std::chrono::steady_clock::now();
                std::cout << "The robot starts the 20 seconds of sanding.\n";
                while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 20) {
                    // Calculate elapsed time in seconds
                    auto elapsedTime = std::chrono::steady_clock::now() - start_time;
                    double elapsedSeconds = std::chrono::duration_cast<std::chrono::duration<double>>(elapsedTime).count();

                    // Update pose_request using the elapsed time
                    pose_request->x = start_pos_x + amplitude * std::sin(2 * M_PI * frequency * elapsedSeconds);
                    pose_request->y = pose_request->y;
                    pose_request->z = pose_request->z;
                    pose_request->roll = pose_request->roll;
                    pose_request->pitch = pose_request->pitch;
                    pose_request->yaw = pose_request->yaw;

                    // Publish the pose
                    publish_pose(pose_client, pose_request, node);

                    // Print the Y value and the elapsed time
                    //std::cout << "pose_request-->y: " << pose_request->y << std::endl;
                    std::cout << "Elapsed time (seconds): " << elapsedSeconds << std::endl;
                }
                force_request->x_force = 0.0;
                force_request->y_force = 0.0;
                force_request->z_force = 0.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                publish_force(force_client, force_request, node);
                std::cout << "Force control deactivated, only impedance control now.\n";
                pose_request->x = 0.5;
                pose_request->y = 0.0;
                pose_request->z = 0.4;
                pose_request->roll = M_PI;
                pose_request->pitch = 0.0;
                pose_request->yaw = -M_PI_2;
                publish_pose(pose_client,pose_request,node);
                std::cout << "Robot moves out of the way to show the result of his work.\n";
                start_time = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 10.0){
                }
                break;
                
            }
            case ',':{                
                gripper_command = "close";
                break;
            }
            case '.':{                
                gripper_command = "open";
                break;
            }
            // Huge part here is currently commented out because the robot struggled with it due to not having accurate enough positional control.
            case '-':{
                
                std::cout << "Screw routine started. The robot will wait for 5 seconds.\n";
                auto start_time = std::chrono::steady_clock::now();
                auto elapsedTime = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 5.0){
                }       //wait 5 seconds
                /*
                pose_request->x = 0.45;
                pose_request->y = 0.298;
                pose_request->z = 0.176;
                pose_request->roll = M_PI ;
                pose_request->pitch = 0.0;
                pose_request->yaw = -M_PI_2;
                std::cout << "The robot will now move near the target object. He has currently 7 seconds to do so.\n";
                publish_pose(pose_client,pose_request,node);
                start_time = std::chrono::steady_clock::now();
                elapsedTime = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count()  < 7.0){
                }

                //do force shit 
                force_request->x_force = 0.0;
                force_request->y_force = 0.0;
                force_request->z_force = -3.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                std::cout << "We exert a force of -3N in z-direction to create a contact.\n";
                publish_force(force_client, force_request, node);
                start_time = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 5.0){
                }
                force_request->x_force = 0.0;
                force_request->y_force = 0.0;
                force_request->z_force = -7.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                std::cout << "We exert a force of -7N in z-direction to keep contact.\n";
                publish_force(force_client, force_request, node);
                start_time = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 5.0){
                }
                pose_request->x = 0.45;
                pose_request->y = 0.298;
                pose_request->z = 0.176;
                pose_request->roll = M_PI ;
                pose_request->pitch = 0.0;
                pose_request->yaw = M_PI_2;
                std::cout << "Turning the screw by 180 degres.\n";
                publish_pose(pose_client,pose_request,node);
                start_time = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 5.0){
                }
                force_request->x_force = 0.0;
                force_request->y_force = 0.0;
                force_request->z_force = 0.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                std::cout << "We exert no force.\n";
                publish_force(force_client, force_request, node);
                start_time = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 5.0){
                }
                pose_request->x = 0.45;
                pose_request->y = 0.298;
                pose_request->z = 0.176;
                pose_request->roll = M_PI ;
                pose_request->pitch = 0.0;
                pose_request->yaw = M_PI_2;
                std::cout << "The robot should go up.\n";
                publish_pose(pose_client,pose_request,node);
                start_time = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count()  < 5.0){
                }
                pose_request->x = 0.45;
                pose_request->y = 0.298;
                pose_request->z = 0.176;
                pose_request->roll = M_PI ;
                pose_request->pitch = 0.0;
                pose_request->yaw = -M_PI_2;
                std::cout << "The robot should turn back.\n";
                publish_pose(pose_client,pose_request,node);
                start_time = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count()  < 4.0){
                }*/
                /*
                //do force shit 
                force_request->x_force = 0.0;
                force_request->y_force = 0.0;
                force_request->z_force = -3.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                std::cout << "We exert a force of -3N in z-direction to create a contact.\n";
                publish_force(force_client, force_request, node);
                start_time = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 5.0){
                }
                pose_request->x = 0.45;
                pose_request->y = 0.298;
                pose_request->z = 0.176;
                pose_request->roll = M_PI ;
                pose_request->pitch = 0.0;
                pose_request->yaw = M_PI_2;
                std::cout << "Turning the screw by 180 degres.\n";
                publish_pose(pose_client,pose_request,node);
                start_time = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 5.0){
                }
                force_request->x_force = 0.0;
                force_request->y_force = 0.0;
                force_request->z_force = 0.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                std::cout << "We exert no force.\n";
                publish_force(force_client, force_request, node);
                start_time = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 5.0){
                }
                pose_request->x = 0.45;
                pose_request->y = 0.298;
                pose_request->z = 0.176;
                pose_request->roll = M_PI ;
                pose_request->pitch = 0.0;
                pose_request->yaw = M_PI_2;
                std::cout << "The robot should go up.\n";
                publish_pose(pose_client,pose_request,node);
                start_time = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count()  < 5.0){
                }
                pose_request->x = 0.45;
                pose_request->y = 0.298;
                pose_request->z = 0.176;
                pose_request->roll = M_PI ;
                pose_request->pitch = 0.0;
                pose_request->yaw = -M_PI_2;
                std::cout << "The robot should turn back.\n";
                publish_pose(pose_client,pose_request,node);
                start_time = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count()  < 4.0){
                }*/
                //do force shit 
                force_request->x_force = 0.0;
                force_request->y_force = 0.0;
                force_request->z_force = -3.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                std::cout << "We exert a force of -3N in z-direction to create a contact.\n";
                publish_force(force_client, force_request, node);
                start_time = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 10.0){
                }
                force_request->x_force = 0.0;
                force_request->y_force = 0.0;
                force_request->z_force = -7.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                std::cout << "We exert a force of -7N in z-direction to keep contact.\n";
                publish_force(force_client, force_request, node);
                start_time = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 10.0){
                }
                //do force shit 
                force_request->x_force = 0.0;
                force_request->y_force = 0.0;
                force_request->z_force = -7.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = -3;
                std::cout << "We exert a force of -3N in z-direction and a torque for 1 second to fasten the screw.\n";
                publish_force(force_client, force_request, node);
                start_time = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 1.0){
                }
                
                force_request->x_force = 0.0;
                force_request->y_force = 0.0;
                force_request->z_force = 0.0;
                force_request->x_torque = 0.0;
                force_request->y_torque = 0.0;
                force_request->z_torque = 0.0;
                std::cout << "We exert no more forces and torques.\n";
                publish_force(force_client, force_request, node);
                start_time = std::chrono::steady_clock::now();
                while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 5.0){
                }
                break;
                
            }
            case '8':{                
                gripper_command = "sanding block"; //sanding block
                break;
            }
            case '9':{                
                gripper_command = "hex wrench"; //hex wrench
                break;
            }
            default:{
                std::cout << "Invalid selection, please try again\n";
                break;
            }
        }
        std::cout<< "Switch has ended"<< std::endl;

        publish_pose(pose_client,pose_request,node);
        std::cout<< "Pose was sent"<< std::endl;
        publish_gripper_task(gripper_command_publisher_, gripper_command, node);
        std::cout<< "Succcessfully published both"<< std::endl;
        /*      //commented because I don't need this part
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
            
        }*/
    }    
    rclcpp::shutdown();
    return 0;
}