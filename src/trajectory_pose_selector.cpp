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
        ament_index_cpp::get_package_share_directory("franka_description")
        + "/robots/fr3/fr3.urdf";

    pinocchio::urdf::buildModel(urdf_path, model_);

    data_ = pinocchio::Data(model_);

    ee_frame_id_ = model_.getFrameId("fr3_hand");

    std::cout << "Loaded robot model\n";
    std::cout << "nq = " << model_.nq << "\n";
    std::cout << "nv = " << model_.nv << "\n";
    std::cout << "EE frame ID = " << ee_frame_id_ << "\n";
}

double TrajectorySelector::distance(const std::array<double, 3>& a, const std::array<double, 3>& b)
{
    return std::sqrt(
        std::pow(a[0] - b[0], 2) +
        std::pow(a[1] - b[1], 2) +
        std::pow(a[2] - b[2], 2)
    );
}

double TrajectorySelector::geodesic_distance(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2)
{
    Eigen::Quaterniond quat_product = q1.inverse() * q2;
    return 2.0 * std::acos(std::abs(quat_product.w()));
}

std::array<double, 3> TrajectorySelector::generate_position()
{
    std::array<double, 3> position;

    do {
        position[0] = uniform_(generator_);
        position[1] = uniform_(generator_);
        position[2] = uniform_(generator_);
    } while (
        std::sqrt(
            std::pow(position[0], 2) + 
            std::pow(position[1], 2) + 
            std::pow(position[2], 2)
        ) > 0.75
    );

    return position;
}

std::array<double, 3> TrajectorySelector::generate_position_within_distance(
    const std::array<double, 3>& previous_position, 
    double target_distance, 
    double tolerance)
{
    std::array<double, 3> position;

    do {
        position = generate_position();
    } while (
        std::abs(distance(previous_position, position) - target_distance) > tolerance
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

std::array<Eigen::Quaterniond> TrajectorySelector::choose_orientation_within_distance( 
    const std::array<Eigen::Quaterniond> filtered_orientations,
    const std::array<double, 3>& previous_orientation,
    const double angle_distance, 
    const double angle_tolerance)
{

}

bool TrajectorySelector::solve_ik(
    const CartesianPose& desired_pose,
    Eigen::VectorXd& q)
{
    const int max_iterations = 1000;
    const double tolerance = 1e-5;
    const double step_size = 0.1;
    const double damping = 0.01;

    for (int iteration = 0; iteration < max_iterations; ++iteration)
    {
        pinocchio::forwardKinematics(model_, data_, q);

        pinocchio::updateFramePlacement(
            model_,
            data_,
            ee_frame_id_);

        const pinocchio::SE3& current_pose =
            data_.oMf[ee_frame_id_];

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
        if (q_error.w() < 0.0)
        {
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

        if (error.norm() < tolerance)
        {
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



double TrajectorySelector::manipulability(
        const Eigen::Matrix<double, 6, 7>& J)
{
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 7>> svd(
        J,
        Eigen::ComputeThinU | Eigen::ComputeThinV);

    double w = 1.0;

    for (int i = 0; i < 6; ++i)
    {
        w *= svd.singularValues()(i);
    }

    return w;
}



std::vector<CartesianPose> TrajectorySelector::filter_poses(
    const Eigen::Vector3d& position,
    int number_of_orientations,
    int number_to_keep)
{
    struct Candidate
    {
        CartesianPose pose;
        Eigen::VectorXd q;
        double manipulability;
    };

    std::vector<Candidate> candidates;

    auto orientations =
        generate_orientations(number_of_orientations);

    for (const auto& orientation : orientations)
    {
        CartesianPose pose;
        pose.position = position;
        pose.orientation = orientation;

        Eigen::VectorXd q =
            Eigen::VectorXd::Zero(model_.nq);

        // TODO:
        // replace this with your actual reasonable
        // initial FR3 configuration
        q[7] = 0.04;
        q[8] = 0.04;

        if (!solve_ik(pose, q))
        {
            continue;
        }

        Eigen::Matrix<double, 6, 9> J_full;

        pinocchio::computeFrameJacobian(
            model_,
            data_,
            q,
            ee_frame_id_,
            pinocchio::LOCAL_WORLD_ALIGNED,
            J_full);

        Eigen::Matrix<double, 6, 7> J =
            J_full.leftCols<7>();

        double w = manipulability(J);

        candidates.push_back({
            pose,
            q,
            w
        });
    }

    // Highest manipulability first
    std::sort(
        candidates.begin(),
        candidates.end(),
        [](const Candidate& a, const Candidate& b)
        {
            return a.manipulability >
                   b.manipulability;
        });

    std::vector<CartesianPose> result;

    int count =
        std::min(
            number_to_keep,
            static_cast<int>(candidates.size()));

    for (int i = 0; i < count; ++i)
    {
        result.push_back(candidates[i].pose);

        std::cout
            << "Candidate " << i
            << " manipulability = "
            << candidates[i].manipulability
            << "\n";
    }

    return result;
}


/*
int main(){
    // Load robot model
    const std::string urdf_path =
        ament_index_cpp::get_package_share_directory("franka_description") +
        "/robots/fr3/fr3.urdf";

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);

    pinocchio::Data data(model);

    std::cout << "nq = " << model.nq << std::endl;
    std::cout << "nv = " << model.nv << std::endl;

    int ee_frame_id = model.getFrameId("fr3_hand");

    
    // Define candidate positions
    Eigen::Vector3d p_des = {0.4, 0.0, 0.4};

    Eigen::VectorXd initial_q = {-0.016473, -0.82876, 0.00329376, -2.60491, 0.00381832, 1.76963, 0.676789, 0, 0};

    // Generate orientations

    // Solve IK for joint configurations

    // Evaluate Jacobian

    // Evaluate manipulability


    // Select poses

    // Save results

    return 0;
}
*/