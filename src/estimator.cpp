#include "estimator.hpp"

EstimatorNode::EstimatorNode() : Node("estimator")
{
    m_actuator_feedback_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "/actuator_feedback", 10, std::bind(&EstimatorNode::actuatorFeedbackCallback, this, std::placeholders::_1));
    
    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/sim_ground_truth", 10, std::bind(&EstimatorNode::odomCallback, this, std::placeholders::_1));

    m_estimate_pub = this->create_publisher<nav_msgs::msg::Odometry>("/estimate_odom", 10);

    m_timer = this->create_wall_timer(
        std::chrono::duration<double>(dt), std::bind(&EstimatorNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "EstimatorNode initialized");
}

// Update actuator feedback
void EstimatorNode::actuatorFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state)
{
    m_actuator_feedback = getInputVector(*joint_state);
}

// Update ground truth
void EstimatorNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    m_ground_truth = getStateVector(*odom);
}

// Add a pose to trajectroies at every timer callback
void EstimatorNode::timerCallback()
{
    if (m_ground_truth.size() == 0)
    {
        return;
    }
    
    m_ground_truth_states.push_back(m_ground_truth);
    Eigen::VectorXd estimated_pose = estimatePose();
    publishOdom(estimated_pose);
    m_estimated_states.push_back(estimated_pose);
}

// Estimate the Vehicle State based off actuator feedback
Eigen::VectorXd EstimatorNode::estimatePose()
{

    if (m_estimated_states.size() == 0)
    {
        return m_ground_truth;
    }

    Eigen::VectorXd prev_state = m_estimated_states.back();

    double theta = prev_state(2);
    double v = prev_state(3);
    double omega = prev_state(4);

    Eigen::Matrix<double, 5, 5> A;
    A << 1, 0, 0, cos(theta) * dt, 0,
         0, 1, 0, sin(theta) * dt, 0,
         0, 0, 1, 0, dt,
         0, 0, 0, 1, 0,
         0, 0, 0, 0, 1;

    double v_wheels = m_actuator_feedback(0);
    double phi = m_actuator_feedback(1);

    double v_instant = r * 0.5 * (v_wheels);
    double omega_instant = v_instant * tan(phi) / L;

    Eigen::VectorXd u(5);
    u.setZero();
    u(3) = v_instant - v;
    u(4) = omega_instant - omega;

    return (A * prev_state) + u;
}



Eigen::VectorXd EstimatorNode::getStateVector(nav_msgs::msg::Odometry odom)
{
    Eigen::VectorXd state(5);
    state(0) = odom.pose.pose.position.x;
    state(1) = odom.pose.pose.position.y;
    state(2) = normalizeAngle(tf2::getYaw(odom.pose.pose.orientation));
    state(3) = odom.twist.twist.linear.x * cos(state(2)) + odom.twist.twist.linear.y * sin(state(2));
    state(4) = odom.twist.twist.angular.z;

    return state;
}

Eigen::VectorXd EstimatorNode::getInputVector(sensor_msgs::msg::JointState joint_state)
{
    Eigen::VectorXd input(2);
    input(0) = joint_state.velocity[0] + joint_state.velocity[1] * 0.5; // average angular velocity of back wheels in rad/sec
    input(1) = joint_state.position[5]; // front steer position in radians

    return input;
}

void EstimatorNode::publishOdom(Eigen::VectorXd state)
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

    odom.twist.twist.linear.x = state(3);
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = state(4);

    m_estimate_pub->publish(odom);
}

double EstimatorNode::normalizeAngle(double angle)
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

// Save trajectorys to file
// EstimatorNode::~EstimatorNode()
// {
// }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EstimatorNode>());
    rclcpp::shutdown();
    return 0;
}

// double v = m_actuator_feedback(0);
// double v2 = m_actuator_feedback(1);
// double v3 = m_actuator_feedback(2);
// double v4 = m_actuator_feedback(3);
// double phi_f = m_actuator_feedback(4);
// double phi_r = m_actuator_feedback(5);
// // Fill in the fourth row (index 3) corresponding to v[k+1]
// B(3, 0) = (r / 4.0) * std::cos(phi_f);
// B(3, 1) = (r / 4.0) * std::cos(phi_f);
// B(3, 2) = (r / 4.0) * std::cos(phi_r);
// B(3, 3) = (r / 4.0) * std::cos(phi_r);
// B(3, 4) = -(r / 4.0) * (v1 + v2) * std::sin(phi_f);
// B(3, 5) = -(r / 4.0) * (v3 + v4) * std::sin(phi_r);

// // Fill in the fifth row (index 4) corresponding to omega[k+1]
// B(4, 0) = (r / (2.0 * L)) * std::sin(phi_f);
// B(4, 1) = (r / (2.0 * L)) * std::sin(phi_f);
// B(4, 2) = -(r / (2.0 * L)) * std::sin(phi_r);
// B(4, 3) = -(r / (2.0 * L)) * std::sin(phi_r);
// B(4, 4) = (r / (2.0 * L)) * (v1 + v2) * std::cos(phi_f);
// B(4, 5) = -(r / (2.0 * L)) * (v3 + v4) * std::cos(phi_r);

// Eigen::VectorXd modifier(5);
// modifier << 0, 0, 0, (r / 2.0) * ((v1 + v2) * std::cos(phi_f) + (v3 + v4) * std::cos(phi_r)), (r / (2.0 * L)) * ((v1 + v2) * std::sin(phi_f) - (v3 + v4) * std::sin(phi_r));

// Eigen::VectorXd modifier(5);
// modifier(0) = 0;
// modifier(1) = 0;
// modifier(2) = 0;
// modifier(3) = (r / 2.0) * ((v1 + v2) * std::cos(phi_f) + (v3 + v4) * std::cos(phi_r));
// modifier(4) = (r / (2.0 * L)) * ((v1 + v2) * std::sin(phi_f) - (v3 + v4) * std::sin(phi_r));

// RCLCPP_INFO(this->get_logger(), "Modifier: [%f, %f, %f, %f, %f]", modifier(0), modifier(1), modifier(2), modifier(3), modifier(4));

// Eigen::VectorXd input(6);
// input(0) = joint_state.velocity[4]; // front left wheel
// input(1) = joint_state.velocity[3]; // front right wheel
// input(2) = joint_state.velocity[1]; // rear left wheel
// input(3) = joint_state.velocity[0]; // rear right wheel
// input(4) = joint_state.position[5]; // front steer
// input(5) = joint_state.position[2]; // rear steer