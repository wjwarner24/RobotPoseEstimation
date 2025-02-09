#ifndef ESTIMATOR_HPP
#define ESTIMATOR_HPP


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <vector>

struct VehicleState
{
    double x;
    double y;
    double theta;
    double Vx;
    double Vy;
    double omega;
};

    
class EstimatorNode : public rclcpp::Node
{
public:
        
    EstimatorNode();

    ~EstimatorNode();

    void actuatorFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state);

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

    // Gets a vehicle state from an odometry message
    VehicleState getState(nav_msgs::msg::Odometry odom);

private:

    // Sub to actuator feedback
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_actuator_feedback_sub;

    // Sub to odometry
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;

    // Timer
    rclcpp::TimerBase::SharedPtr m_timer;

    // Ground Truth States
    std::vector<VehicleState> m_ground_truth_states;

    // Estimated States
    std::vector<VehicleState> m_estimated_states;

    VehicleState m_ground_truth;
    VehicleState m_estimated;
    sensor_msgs::msg::JointState m_actuator_feedback;

    // Vehicle params
    double wheel_radius = 0.05;
    double wheel_base = 0.5;
    double track_width = 0.5;

    
};

#endif
