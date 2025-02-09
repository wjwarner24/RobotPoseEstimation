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

// struct VehicleState
// {
//     double x;
//     double y;
//     double theta;
//     // double Vx;
//     // double Vy;
//     double v;
//     double omega;
// };

    
class EstimatorNode : public rclcpp::Node
{
public:
        
    EstimatorNode();

    //~EstimatorNode();

    void actuatorFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state);

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

    void timerCallback();

    Eigen::VectorXd getStateVector(nav_msgs::msg::Odometry odom);

    Eigen::VectorXd getInputVector(sensor_msgs::msg::JointState joint_state);

    Eigen::VectorXd estimatePose();

    void publishOdom(Eigen::VectorXd state);

    double normalizeAngle(double angle);

private:

    // Sub to actuator feedback
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_actuator_feedback_sub;

    // Sub to odometry
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;

    // Timer
    rclcpp::TimerBase::SharedPtr m_timer;

    // Estimate Publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_estimate_pub;

    // Ground Truth States
    std::vector<Eigen::VectorXd> m_ground_truth_states;

    // Estimated States
    std::vector<Eigen::VectorXd> m_estimated_states;

    // Current GT odom
    Eigen::VectorXd m_ground_truth;
    Eigen::VectorXd m_actuator_feedback;

    // Vehicle params
    double r = 0.05; // wheel radius in meters
    double L = 0.4; // wheelbase in meters (distance between front and rear axles)

    // time step
    double dt = 0.01;

};

#endif
