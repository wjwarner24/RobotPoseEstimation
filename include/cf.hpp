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

class CFNode : public rclcpp::Node
{
public:
    CFNode();

    void actuatorFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state);

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

    void timerCallback();

    Vector3d getStateVector(nav_msgs::msg::Odometry odom);

    Vector2d getInputVector(sensor_msgs::msg::JointState joint_state);

    void publishOdom(Vector3d state);

    double normalizeAngle(double angle);

    Vector3d processModel(const Vector3d &x, const Vector2d &u, double dt);

private:

    Vector3d est_state;
    Vector2d m_actuator_feedback;
    Vector3d m_ground_truth;

    // Sub to actuator feedback
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_actuator_feedback_sub;
    // Sub to odometry
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
    // Timer
    rclcpp::TimerBase::SharedPtr m_timer;
    // Estimate Publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_estimate_pub;

    double dt = 0.01;

    double alpha = 0.8;

        // randomm noise
    static std::default_random_engine generator;
    static std::normal_distribution<double> dist;

    // Vehicle params
    double r = 0.05; // wheel radius in meters
    double L = 0.4;  // wheelbase in meters (distance between front and rear axles)
};