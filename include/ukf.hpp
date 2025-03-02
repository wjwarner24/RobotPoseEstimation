#ifndef ESTIMATOR_HPP
#define ESTIMATOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include <random>

using namespace Eigen;

struct UKF
{
    Vector3d x_; // State: [x, y, theta]
    Matrix3d P_; // State covariance
    Matrix3d Q_; // Process noise covariance
    Matrix3d R_; // Measurement noise covariance
};

class UKFNode : public rclcpp::Node
{
public:
    UKFNode();

    void actuatorFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state);

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

    void timerCallback();

    Vector3d getStateVector(nav_msgs::msg::Odometry odom);

    Vector2d getInputVector(sensor_msgs::msg::JointState joint_state);

    void publishOdom(Vector3d state);

    //void publishNoisyOdom(Eigen::Vector3d state);

    double normalizeAngle(double angle);

    //Vector3d predictState(const Vector3d &x, const Vector2d &u, double dt);

    //Matrix3d computeJacobianF(const Vector3d &x, const Vector2d &u, double dt);

    //Matrix<double, 3, 2> computeJacobianB(const Vector3d &x, const Vector2d &u, double dt);

    void predict(const Vector2d &u, double dt);

    void update(const Vector3d &z);

    Vector3d processModel(const Vector3d &x, const Vector2d &u, double dt);

    private :
    // Sub to actuator feedback
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_actuator_feedback_sub;

    // Sub to odometry
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;

    // Timer
    rclcpp::TimerBase::SharedPtr m_timer;

    // Estimate Publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_estimate_pub;

    // Noisy ground truth publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_noisy_pub;

    // Current measurements from sim
    Eigen::Vector3d m_ground_truth;
    Eigen::Vector2d m_actuator_feedback;

    // Vehicle params
    double r = 0.05; // wheel radius in meters
    double L = 0.4;  // wheelbase in meters (distance between front and rear axles)

    // time step
    double dt = 0.01;

    // UKF
    UKF ukf;

    // Random number generator
    static std::default_random_engine generator;
    static std::normal_distribution<double> dist;

    const double PI = 3.141592653589793;

    int n_sigma_;   // Number of sigma points
    double alpha_;  // Primary scaling parameter
    double kappa_;  // Secondary scaling parameter
    double beta_;   // Used to incorporate prior knowledge of the distribution (optimal value is 2 for Gaussian)
    double lambda_; // Composite scaling parameter

    VectorXd weights_m_; // Weights for the mean
    VectorXd weights_c_; // Weights for the covariance

    const int n_x = 3;
    const int n_z = 3;
};

#endif
