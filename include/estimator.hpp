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

struct EKF {
    Vector3d x; // State: [x, y, theta]
    Matrix3d P; // State covariance
    Matrix2d Q; // Process noise covariance
    Matrix3d R; // Measurement noise covariance
};

    
class EstimatorNode : public rclcpp::Node
{
public:
        
    EstimatorNode();

    void actuatorFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state);

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

    void timerCallback();

    Vector3d getStateVector(nav_msgs::msg::Odometry odom);

    Vector2d getInputVector(sensor_msgs::msg::JointState joint_state);

    void publishOdom(Vector3d state);

    void publishNoisyOdom(Eigen::Vector3d state);

    double normalizeAngle(double angle);

    Vector3d predictState(const Vector3d &x, const Vector2d &u, double dt);

    Matrix3d computeJacobianF(const Vector3d &x, const Vector2d &u, double dt);

    Matrix<double, 3, 2> computeJacobianB(const Vector3d &x, const Vector2d &u, double dt);

    void predict(EKF &ekf, const Vector2d &u, double dt, const Matrix2d &Q);

    void update(EKF &ekf, const Vector3d &z, const Matrix3d &R);

private:

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
    double L = 0.4; // wheelbase in meters (distance between front and rear axles)

    // time step
    double dt = 0.01;

    // EKF
    EKF ekf;

    // Random number generator
    static std::default_random_engine generator;
    static std::normal_distribution<double> dist;
};

#endif
