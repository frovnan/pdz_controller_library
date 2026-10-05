#include "pdz_controller_library/trajectory_pose_selector.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/se3.hpp>


TrajectorySelector::TrajectorySelector()
    : generator_(std::random_device{}()),
      uniform_(-0.5, 0.5)
{
    const std::string urdf_path =
        ament_index_cpp::get_package_share_directory("pdz_controller_library")
        + "/urdf/robot.urdf";

    pinocchio::urdf::buildModel(urdf_path, model_);

    data_ = pinocchio::Data(model_);

    ee_frame_id_ = model_.getFrameId("fr3_hand");

    std::cout << "Loaded robot model\n";
    std::cout << "nq = " << model_.nq << "\n";
    std::cout << "nv = " << model_.nv << "\n";
    std::cout << "EE frame ID = " << ee_frame_id_ << "\n";
}

double TrajectorySelector::geodesic_distance(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2)
{
    // Geodesic distance between two quaternions
    Eigen::Quaterniond quat_product = q1.inverse() * q2;
    return 2.0 * std::acos(std::abs(quat_product.w()));
}

Eigen::Vector3d TrajectorySelector::generate_position()
{
    Eigen::Vector3d position;
    
    do {
        // Generate a random position...
        position[0] = uniform_(generator_);
        position[1] = uniform_(generator_);
        position[2] = uniform_(generator_);
    } while (
        // ...within a sphere of radius 0.75m, and...
        position.norm() > 0.75 ||

        // ...beyond a sphere of radius 0.25m, and...
        position.norm() < 0.25 ||

        // ...ensure the z-coordinate is above 0.1m to avoid collisions with the table
        position[2] < 0.1 
    );

    return position;
}

Eigen::Vector3d TrajectorySelector::generate_position_within_distance(
    const Eigen::Vector3d& previous_position, 
    double target_distance, 
    double tolerance)
{
    Eigen::Vector3d position;

    do {
        // Generate a random position within the constrained region...
        position = generate_position();
    } while (
        // ...that is within the specified distance from the previous position
        std::abs((previous_position - position).norm() - target_distance) > tolerance
    );

    return position;
}


std::vector<Eigen::Quaterniond> TrajectorySelector::generate_orientations(int number_of_orientations)
{
    std::vector<Eigen::Quaterniond> orientations;

    orientations.reserve(number_of_orientations);

    std::mt19937 generator(42);

    std::uniform_real_distribution<double> uniform_(0.0, 1.0);

    for (int i = 0; i < number_of_orientations; ++i)
    {
        double u1 = uniform_(generator);
        double u2 = uniform_(generator);
        double u3 = uniform_(generator);

        Eigen::Quaterniond q(
            std::sqrt(1.0 - u1) * std::sin(2.0 * M_PI * u2),
            std::sqrt(1.0 - u1) * std::cos(2.0 * M_PI * u2),
            std::sqrt(u1) * std::sin(2.0 * M_PI * u3),
            std::sqrt(u1) * std::cos(2.0 * M_PI * u3));

        orientations.push_back(q);
    }

    return orientations;
}


bool TrajectorySelector::solve_ik(const CartesianPose& desired_pose, Eigen::VectorXd& q)
{
    const int max_iterations = 1000;
    const double tolerance = 1e-5;
    const double step_size = 0.1;
    const double damping = 0.01;

    for (int iteration = 0; iteration < max_iterations; ++iteration)
    {
        pinocchio::forwardKinematics(model_, data_, q);

        pinocchio::updateFramePlacement(model_, data_, ee_frame_id_);

        const pinocchio::SE3& current_pose = data_.oMf[ee_frame_id_];

        // --------------------------------------------------
        // Position error
        // --------------------------------------------------
        Eigen::Vector3d position_error = desired_pose.position - current_pose.translation();

        // --------------------------------------------------
        // Orientation error
        // --------------------------------------------------
        Eigen::Quaterniond q_current(current_pose.rotation());

        Eigen::Quaterniond q_error = q_current.inverse() * desired_pose.orientation;

        // Make sure we take the shortest quaternion path
        if (q_error.w() < 0.0){
            q_error.coeffs() *= -1.0;
        }

        Eigen::Vector3d orientation_error = 2.0 * q_error.vec();

        // --------------------------------------------------
        // Combined 6D error
        // --------------------------------------------------
        Eigen::Matrix<double, 6, 1> error;

        error.head<3>() = position_error;
        error.tail<3>() = orientation_error;

        // --------------------------------------------------
        // Check convergence
        // --------------------------------------------------
        if (error.norm() < tolerance){
            return true;
        }

        // --------------------------------------------------
        // Jacobian
        // --------------------------------------------------
        Eigen::Matrix<double, 6, 9> J_full;

        pinocchio::computeFrameJacobian(model_, data_, q, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_full);

        // Only use the seven arm joints
        Eigen::Matrix<double, 6, 7> J = J_full.leftCols<7>();

        // --------------------------------------------------
        // Damped pseudoinverse
        // --------------------------------------------------
        Eigen::Matrix<double, 7, 6> J_pinv = J.transpose() * (J * J.transpose() + damping * damping * Eigen::Matrix<double, 6, 6>::Identity()).inverse();

        // --------------------------------------------------
        // Joint update
        // --------------------------------------------------
        Eigen::Matrix<double, 7, 1> dq = J_pinv * error;

        q.head<7>() += step_size * dq;
    }

    return false;
}


double TrajectorySelector::manipulability(const Eigen::Matrix<double, 6, 7>& J)
{
    /*
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 7>> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);

    double mu = 1.0;

    for (int i = 0; i < 6; ++i)
    {
        mu *= svd.singularValues()(i);
    }
    */

    // Using the sqrt(det(J * J^T)) to compute manipulability, as proposed by Yoshikawa (1985)
    double mu = std::sqrt((J * J.transpose()).determinant());

    return mu;
}


std::vector<Candidate> TrajectorySelector::filter_poses(
    const Eigen::Vector3d& position,
    const std::vector<Eigen::Quaterniond>& orientations,
    int number_to_keep,
    const Eigen::VectorXd& initial_q)
{
    std::vector<Candidate> candidates;

    for (const auto& orientation : orientations)
    {
        // Desired EE pose
        CartesianPose pose;
        pose.position = position;
        pose.orientation = orientation;

        // Initial guess for IK solver (= previous joint configuration)
        Eigen::VectorXd q = initial_q;

        // Solve IK for the desired pose
        if (!solve_ik(pose, q)){
            continue;
        }
        // Note: q is now the joint configuration that achieves the desired pose (IK solution)


        // Compute Jacobian at IK solution
        Eigen::Matrix<double, 6, 9> J_full;

        pinocchio::computeFrameJacobian(model_, data_, q, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_full);

        Eigen::Matrix<double, 6, 7> J = J_full.leftCols<7>();

        // Compute manipulability at IK solution
        double mu = manipulability(J);

        // Store desired pose, corresponding joint angles, and manipulability at that joint configuration
        candidates.push_back({
            pose,
            q,
            mu
        });
    }

    // Highest manipulability first
    std::sort(
        candidates.begin(), candidates.end(),
        [](const Candidate& a, const Candidate& b){
            return a.manipulability > b.manipulability;
        }
    );

    std::vector<Candidate> result;

    int count = std::min(number_to_keep, static_cast<int>(candidates.size()));

    for (int i = 0; i < count; ++i)
    {
        result.push_back(candidates[i]);

        std::cout
            << "Candidate " << i
            << " manipulability = "
            << candidates[i].manipulability
            << "\n";
    }

    return result;
}



Candidate TrajectorySelector::select_random_candidate(const std::vector<Candidate>& candidates, const std::vector<int>& valid_indices)
{
    std::uniform_int_distribution<int> distribution(0, static_cast<int>(valid_indices.size()) - 1);

    int index = valid_indices[distribution(generator_)];

    return candidates[index];
}

Candidate TrajectorySelector::select_candidate_with_angle_constraint( 
    const std::vector<Candidate>& candidates,
    const Eigen::Quaterniond& previous_orientation,
    const double angle_distance, 
    const double angle_tolerance)
{
    // Pick indices of candidates that satisfy the angle constraint
    std::vector<int> valid_indices;

    std::cout << "valid indices: ";
    for (unsigned int i = 0; i < candidates.size(); ++i){
        double angle_diff = geodesic_distance(previous_orientation, candidates[i].pose.orientation);

        if (std::abs(angle_diff - angle_distance) <= angle_tolerance){
            std::cout << i << ", ";
            valid_indices.push_back(i);
        }
    }
    std::cout << "\n";

    // Pick random candidate from valid candidates
    Candidate candidate = select_random_candidate(candidates, valid_indices);

    return candidate;
}


std::array<double, 6> TrajectorySelector::quaternion_to_euler(const CartesianPose& pose)
{
    Eigen::Vector3d euler_angles = pose.orientation.toRotationMatrix().eulerAngles(0, 1, 2);

    return {pose.position[0], pose.position[1], pose.position[2], euler_angles[0], euler_angles[1], euler_angles[2]};
}