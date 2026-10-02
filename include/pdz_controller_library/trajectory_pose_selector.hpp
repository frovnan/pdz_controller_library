#pragma once

#include <string>
#include <vector>
#include <random>

#include <Eigen/Dense>

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>

struct CartesianPose
{
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

class TrajectorySelector
{
    public:

        explicit TrajectorySelector();

        std::array<double, 3> generate_position();
        std::array<double, 3> generate_position_within_distance(
            const std::array<double, 3>& previous_position, 
            const double target_distance, 
            const double tolerance);
        std::vector<Eigen::Quaterniond> generate_orientations(int number_of_orientations);
        std::array<Eigen::Quaterniond> choose_orientation_within_distance(
            const std::array<Eigen::Quaterniond> filtered_orientations,
            const std::array<double, 3>& previous_orientation,
            const double angle_distance,
            const double angle_tolerance);

        std::vector<CartesianPose> filter_poses(
            const Eigen::Vector3d& position,
            int number_of_orientations,
            int number_to_keep);

    private:

        double manipulability(const Eigen::Matrix<double, 6, 7>& J);

        bool solve_ik(const CartesianPose& desired_pose, Eigen::VectorXd& q);
        
        double distance(const std::array<double, 3>& a, const std::array<double, 3>& b);
        double geodesic_distance(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2);

        pinocchio::Model model_;
        pinocchio::Data data_;

        int ee_frame_id_;

        // Random number generator for reproducibility
        std::mt19937 generator_;
        std::uniform_real_distribution<double> uniform_;
};

