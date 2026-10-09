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

        std::vector<CartesianPose> generate_poses(
            const Candidate& initial_pose_data, int number_of_points);

        std::array<double, 6> quaternion_to_euler(
            const CartesianPose& pose);

    private:

        Eigen::Vector3d generate_position();


        bool segment_clear_of_base(
            const Eigen::Vector3d& start, 
            const Eigen::Vector3d& end);


        Eigen::Vector3d generate_position_with_constraints(
            const Eigen::Vector3d& previous_position, 
            const double target_distance, 
            const double tolerance);


        std::vector<Eigen::Quaterniond> generate_orientations(
            int number_of_orientations);


        std::vector<Eigen::Quaterniond> generate_orientations_within_distance(
            int number_of_orientations,
            const Eigen::Quaterniond& previous_orientation,
            const double angle_distance,
            const double angle_tolerance);

        bool solve_ik(
            const CartesianPose& desired_pose, 
            Eigen::VectorXd& q);


        double manipulability(
            const Eigen::Matrix<double, 6, 7>& J);


        std::vector<Candidate> filter_poses(
            const Eigen::Vector3d& position,
            const std::vector<Eigen::Quaterniond>& orientations,
            int number_to_keep,
            const Eigen::VectorXd& initial_q);


        Candidate select_random_candidate(
            const std::vector<Candidate>& candidates);


        pinocchio::Model model_;
        pinocchio::Data data_;

        int ee_frame_id_;

        // Random number generator for reproducibility
        std::mt19937 generator_;
        std::uniform_real_distribution<double> uniform_;
};

