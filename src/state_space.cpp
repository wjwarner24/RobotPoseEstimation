#include "state_space.hpp"

SSNode::SSNode() : Node("state_space")
{
    m_actuator_feedback_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "/actuator_feedback", 10, std::bind(&SSNode::actuatorFeedbackCallback, this, std::placeholders::_1));
    
   

    m_estimate_pub = this->create_publisher<nav_msgs::msg::Odometry>("/estimate_odom", 10);

    

    m_timer = this->create_wall_timer(
        std::chrono::duration<double>(dt), std::bind(&SSNode::timerCallback, this));


    est_state << 0, 0, 0; // Initial state with no uncertainty
    RCLCPP_INFO(this->get_logger(), "SSNode initialized");
}

// Update actuator feedback
void SSNode::actuatorFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state)
{
    m_actuator_feedback = getInputVector(*joint_state);
}

// Gets the input vector from the sim with no added noise
// There will be uncertainty that comes from whether these values are acheived
Eigen::Vector2d SSNode::getInputVector(sensor_msgs::msg::JointState joint_state)
{
    double a = (joint_state.velocity[0] + joint_state.velocity[1]) * 0.5; // average angular velocity of back wheels in rad/sec
    double phi = joint_state.position[5];                                 // front steer position in radians

    Eigen::Vector2d input;
    input(0) = a * r;
    input(1) = (a * r * tan(phi)) / L;

    return input;
}

// normalize angle
double SSNode::normalizeAngle(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2 * M_PI;
    }
    return angle;
}

// Publishes an odom msg with the current estimate
void SSNode::publishOdom(Eigen::Vector3d state)
{
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = state(0);
    odom.pose.pose.position.y = state(1);
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, state(2));
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    m_estimate_pub->publish(odom);
}

// timer callback
void SSNode::timerCallback()
{
    Vector2d u = m_actuator_feedback;

    est_state = processModel(est_state, u, dt);
    est_state(2) = normalizeAngle(est_state(2));

    publishOdom(est_state);
}

// process model
Vector3d SSNode::processModel(const Vector3d &x, const Vector2d &u, double dt)
{
    double v = u(0);
    double w = u(1);
    double theta = x(2);

    Vector3d x_pred;
    x_pred(0) = x(0) + v * cos(theta) * dt;
    x_pred(1) = x(1) + v * sin(theta) * dt;
    x_pred(2) = normalizeAngle(theta + w * dt);
    return x_pred;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SSNode>());
    rclcpp::shutdown();
    return 0;
}