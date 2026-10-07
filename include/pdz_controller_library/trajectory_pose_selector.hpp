#pragma once

#include <string>
#include <vector>
#include <random>

#include <Eigen/Dense>

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>

struct CartesianPose {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

struct Candidate {
    CartesianPose pose;
    Eigen::VectorXd q;
    double manipulability;
};

class TrajectorySelector {
    public:

        explicit TrajectorySelector();

        Eigen::Vector3d generate_position();
        Eigen::Vector3d generate_position_within_distance(
            const Eigen::Vector3d& previous_position, 
            const double target_distance, 
            const double tolerance);

        std::vector<Eigen::Quaterniond> generate_orientations(int number_of_orientations);

        std::vector<Eigen::Quaterniond> generate_orientations_within_distance(
            int number_of_orientations,
            const Eigen::Quaterniond& previous_orientation,
            const double angle_distance,
            const double angle_tolerance);

        std::vector<Candidate> filter_poses(
            const Eigen::Vector3d& position,
            const std::vector<Eigen::Quaterniond>& orientations,
            int number_to_keep,
            const Eigen::VectorXd& initial_q);

        Candidate select_random_candidate(const std::vector<Candidate>& candidates);

        std::array<double, 6> quaternion_to_euler(const CartesianPose& pose);

    private:

        double manipulability(const Eigen::Matrix<double, 6, 7>& J);

        bool solve_ik(const CartesianPose& desired_pose, Eigen::VectorXd& q);

        pinocchio::Model model_;
        pinocchio::Data data_;

        int ee_frame_id_;

        // Random number generator for reproducibility
        std::mt19937 generator_;
        std::uniform_real_distribution<double> uniform_;
};

