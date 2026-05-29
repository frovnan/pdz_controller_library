# pdz_controller_library

## Description

A ROS 2 Humble package that contains various works by students from Pd|Z for the Franka Emika FR3 robot arm. 

These include:

* [Cartesian Impedance Controller](https://github.com/LucasG2001/cartesian_impedance_control)
* [Admittance Controller](https://github.com/LucasG2001/admittance_control)
* Joint Impedance Controller
* [Hybrid Force/Impedance Contoller](https://github.com/krombier/Bachelor_Thesis_Force_Control)
* [Hybrid Impedance/Admittance Controller](https://github.com/enjoyericbu/hybridcontroller)
* [Potential Field Method](https://github.com/anel-b/repulsive_force)
* [Riemann Motion Policy](https://github.com/MatteoBodmer/riemannian_motion_policy_mb)
* [Singularity and Oscillation Avoidance](https://github.com/GeniusT31/src)

## Prerequisites

* Ubuntu 22.04 LTS+ (Linux)
* [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* franka_ros2 >= v2.1.0
* libfranka >= v0.18.0

---

## Download

### Step 1: Clone the repository

Change your current directory in the terminal to **/franka_ros2_ws/src** and clone the repository into this folder.

```bash
cd franka_ros2_ws/src
git clone https://github.com/frovnan/pdz_controller_library.git
cd franka_ros2_ws
colcon build --packages-select pdz_controller_library
source install/setup.bash
```

### Step 2: Modify configuration

Navigate to the 'controllers.yaml' file located in 'franka_bringup/config/', and add the following lines:

```yaml
      cartesian_impedance_controller:
        type: pdz_contoller_library/CartesianImpedanceController

      riemannian_motion_policy:
        type: pdz_contoller_library/RiemannianMotionPolicy
      
      joint_impedance_controller:
        type: pdz_contoller_library/JointImpedanceController
      
      joint_impedance_ik_controller:
        type: pdz_contoller_library/JointImpedanceIkController
      
      joint_impedance_ik_controller:
        type: pdz_contoller_library/JointImpedanceIkController

      impedance_admittance_hybrid_controller:
        type: pdz_controller_library/ImpedanceAdmittanceHybridController
    
      admittance_controller:
        type: pdz_controller_library/AdmittanceController
```

If you plan on using Gazebo, do the same in the 'franka_gazebo_controllers.yaml' inside 'franka_gazebo/config/'.
Further down, you will need to add these lines:

```yaml
/**:
  cartesian_impedance_controller:
    ros__parameters:
      robot_type: "fr3"
      k_gains:
        - 2500.0
        - 2500.0
        - 1000.0
        - 130.0
        - 130.0
        - 45.0
      d_gains:
        - 35.0
        - 35.0
        - 35.0
        - 25.0
        - 25.0
        - 6.0

/**:
  joint_impedance_controller:
    ros__parameters:
      robot_type: "fr3"
      k_gains:
        - 85.0
        - 135.0
        - 90.0
        - 90.0
        - 15.0
        - 8.0
        - 3.0
      d_gains:
        - 14.0
        - 23.0
        - 18.0
        - 18.0
        - 2.5
        - 1.8
        - 0.8

/**:
  joint_impedance_ik_controller:
    ros__parameters:
      robot_type: "fr3"
      k_gains:
        - 600.0
        - 600.0
        - 600.0
        - 600.0
        - 400.0
        - 150.0
        - 50.0
      d_gains:
        - 30.0
        - 30.0
        - 30.0
        - 30.0
        - 15.0
        - 10.0
        - 10.0

/**:
  riemannian_motion_policy:
    ros__parameters:
      robot_type: "fr3"

/**:
  impedance_admittance_hybrid_controller:
    ros__parameters:
      robot_type: "fr3"
  
/**:
  admittance_controller:
    ros__parameters:
      robot_type: "fr3"
```

The numbers under the impedance controllers are the stiffness and damping gains for each task-space DoF (in the case of cartesian impedance) or joint-space DoF (in the case of joint impedace). Change them to suit your needs.

### Step 3: Clone the Messages Package

Clone the [messages_fr3](https://github.com/acaviezel/messages_fr3) package into the 'src' folder of your workspace.

```bash
cd franka_ros2_ws/src
git clone https://github.com/acaviezel/messages_fr3.git 
```

Navigate to the 'CMakeLists.txt' file in the 'messages_fr3' package and add the following line in the 'rosidl_generate_interfaces' section:

```txt
"srv/SetForce.srv"
```

It is needed for the Hybrid Force/Impedance control functionality.

### Step 4: Build the workspace

Change your current directory in the terminal back to  **/franka_ros2_ws** and build the package or the entire workspace.

  ```bash
  cd ..
  colcon build --packages-select pdz_controller_library
  ```

### Step 5: Update '.bashrc' file

In order to not have to source the setup file after every build, you can add the following line at the end of your .bashrc file (which you can access by executing `nano .bashrc` in your home directory): 

```bash
source /home/<user>/franka_ros2_ws/install/setup.bash
```

---

## Usage

To launch a controller, execute the following command:

```bash
ros2 launch pdz_controller_library <launch file name>
```

To run any additional functionalities like e.g. the user_input_client, open a new terminal and enter the following:

```bash
ros2 run pdz_controller_library <node name>
```

##
