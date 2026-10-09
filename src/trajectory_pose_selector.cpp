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
      uniform_(0.0, 1.0)
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

Eigen::Vector3d TrajectorySelector::generate_position()
{
    // Uniform distribution of x,y,z within allowed volume
    const double r_min = 0.40; // Minimum distance to origin
    const double r_max = 0.75; // Maximum distance to origin
    const double z_min = 0.15; // Minimum height to prevent collision with table

    double u1 = uniform_(generator_);
    double u2 = uniform_(generator_);
    double u3 = uniform_(generator_);

    double r = std::cbrt(r_min*r_min*r_min + u1 * (r_max*r_max*r_max - r_min*r_min*r_min)); // Radius

    double phi = 2.0 * M_PI * u2; // Azimuthal angle

    double cos_theta = z_min / r + u3 * (1.0 - z_min / r);

    double sin_theta = std::sqrt(1.0 - cos_theta * cos_theta);

    Eigen::Vector3d position;

    position.x() = r * sin_theta * std::cos(phi);
    position.y() = r * sin_theta * std::sin(phi);
    position.z() = r * cos_theta;

    return position;
}

Eigen::Vector3d TrajectorySelector::generate_position_with_constraints(
    const Eigen::Vector3d& previous_position, 
    double target_distance, 
    double tolerance)
{
    Eigen::Vector3d new_position;

    do {
        // Generate a random position within the constrained region...
        new_position = generate_position();
    } while (
        // ...that is within the specified distance from the previous position
        std::abs((previous_position - new_position).norm() - target_distance) > tolerance &&
        !segment_clear_of_base(previous_position, new_position)
    );

    return new_position;
}

bool TrajectorySelector::segment_clear_of_base(
    const Eigen::Vector3d& A,
    const Eigen::Vector3d& B)
{
    // Approximate robot base as cylinder with base radius and base height
    const double base_radius = 0.075;
    const double clearance = 0.05;
    const double forbidden_radius = base_radius + clearance;

    const double base_height = 0.45;

    // Divide line between A and B into 100 points
    const int samples = 100;

    for (int i = 0; i <= samples; ++i)
    {
        double t = static_cast<double>(i) / samples;

        Eigen::Vector3d p = A + t * (B - A);

        double radial_distance = std::sqrt(p.x() * p.x() + p.y() * p.y());

        // Check whether point is inside cylinder
        if (p.z() < base_height && radial_distance < forbidden_radius)
        {
            return false;
        }
    }

    return true;
}

std::vector<Eigen::Quaterniond> TrajectorySelector::generate_orientations(int number_of_orientations)
{
    std::vector<Eigen::Quaterniond> orientations;

    orientations.reserve(number_of_orientations);

    std::uniform_real_distribution<double> uniform_(0.0, 1.0);

    for (int i = 0; i < number_of_orientations; ++i)
    {
        double u1 = uniform_(generator_);
        double u2 = uniform_(generator_);
        double u3 = uniform_(generator_);

        Eigen::Quaterniond q(
            std::sqrt(1.0 - u1) * std::sin(2.0 * M_PI * u2),
            std::sqrt(1.0 - u1) * std::cos(2.0 * M_PI * u2),
            std::sqrt(u1) * std::sin(2.0 * M_PI * u3),
            std::sqrt(u1) * std::cos(2.0 * M_PI * u3));
        
        q.normalize();
        
        orientations.push_back(q);
    }

    return orientations;
}

std::vector<Eigen::Quaterniond> TrajectorySelector::generate_orientations_within_distance(
    int number_of_orientations,
    const Eigen::Quaterniond& previous_orientation,
    const double angle_distance,
    const double angle_tolerance)
{
    std::vector<Eigen::Quaterniond> orientations;
    orientations.reserve(number_of_orientations);

    std::normal_distribution<double> normal(0.0, 1.0);

    std::uniform_real_distribution<double> angle_distribution(angle_distance - angle_tolerance, angle_distance + angle_tolerance);

    for (int i = 0; i < number_of_orientations; ++i)
    {
        // Random unit rotation axis
        Eigen::Vector3d axis(
            normal(generator_),
            normal(generator_),
            normal(generator_));
        axis.normalize();

        // Random angle within desired range
        double angle = angle_distribution(generator_);

        // Construct relative rotation
        Eigen::AngleAxisd relative_rotation(angle, axis);

        // Apply to previous orientation
        Eigen::Quaterniond q = previous_orientation * Eigen::Quaterniond(relative_rotation);
        q.normalize();

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



Candidate TrajectorySelector::select_random_candidate(const std::vector<Candidate>& candidates)
{
    std::uniform_int_distribution<int> distribution(0, static_cast<int>(candidates.size()) - 1);

    return candidates[distribution(generator_)];
}


std::array<double, 6> TrajectorySelector::quaternion_to_euler(const CartesianPose& pose)
{
    Eigen::Vector3d euler_angles = pose.orientation.toRotationMatrix().eulerAngles(0, 1, 2);

    return {pose.position[0], pose.position[1], pose.position[2], euler_angles[0], euler_angles[1], euler_angles[2]};
}

std::vector<CartesianPose> TrajectorySelector::generate_poses(const Candidate& initial_pose_data, int number_of_points){
    // --------- Parameters -----------
    const double path_length = 0.6; // Length of the path in meters
    const double tolerance = 0.05; // Tolerance for the distance between points

    const double angle_distance = 60 * M_PI / 180.0; // Angle distance in radians
    const double angle_tolerance = 7.5 * M_PI / 180.0; // Angle tolerance in radians

    const int n_orientations = 200; // Number of orientations to generate for each point
    const int number_to_keep = 15; // Number of candidates to keep after filtering based on manipulability

    std::vector<Candidate> candidates;
    std::vector<CartesianPose> poses = {initial_pose_data.pose};

    for (int i = 0; i < number_of_points; ++i) {
        std::vector<Candidate> filtered_poses;
        if (i == 0) {
            auto position = generate_position();
            auto orientations = generate_orientations(n_orientations);
            filtered_poses = filter_poses(position, orientations, number_to_keep, initial_pose_data.q);

        } else {
            auto position = generate_position_with_constraints(candidates[i-1].pose.position, path_length, tolerance);
            auto orientations = generate_orientations_within_distance(n_orientations, candidates[i-1].pose.orientation, angle_distance, angle_tolerance);
            filtered_poses = filter_poses(position, orientations, number_to_keep, candidates[i-1].q);
        }
        auto pose = select_random_candidate(filtered_poses);
        candidates.push_back(pose);
        poses.push_back(pose.pose);

        std::cout << "position: " << pose.pose.position.transpose() << std::endl;
        std::cout << "orientation: " << pose.pose.orientation << std::endl;
    }
    return poses;
}