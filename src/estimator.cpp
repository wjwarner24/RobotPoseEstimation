#include "estimator.hpp"

EstimatorNode::EstimatorNode() : Node("estimator")
{
    m_actuator_feedback_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "/actuator_feedback", 10, std::bind(&EstimatorNode::actuatorFeedbackCallback, this, std::placeholders::_1));
    
    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/sim_ground_truth", 10, std::bind(&EstimatorNode::odomCallback, this, std::placeholders::_1));

    m_timer = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&EstimatorNode::timerCallback, this));
}

// Update actuator feedback
void EstimatorNode::actuatorFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state)
{
    m_actuator_feedback = *joint_state;
}

// Update ground truth
void EstimatorNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    m_ground_truth = getState(*odom);
}

void EstimatorNode::timerCallback()
{
    m_ground_truth_states.push_back(m_ground_truth);
    VehicleState estimated_pose = estimatePose();
    m_estimated_states.push_back(estimated_pose);
}

// Estimate the Vehicle State based off actuator feedback
VehicleState EstimatorNode::estimatePose()
{
    VehicleState state;
    state.x = m_ground_truth.x + m_ground_truth.Vx * 0.1;
    state.y = m_ground_truth.y + m_ground_truth.Vy * 0.1;
    state.theta = m_ground_truth.theta + m_ground_truth.omega * 0.1;
    state.Vx = m_ground_truth.Vx;
    state.Vy = m_ground_truth.Vy;
    state.omega = m_ground_truth.omega;

    return state;
}

VehicleState EstimatorNode::getState(nav_msgs::msg::Odometry odom)
{
    VehicleState state;
    state.x = odom.pose.pose.position.x;
    state.y = odom.pose.pose.position.y;
    state.theta = tf2::getYaw(odom.pose.pose.orientation);
    state.Vx = odom.twist.twist.linear.x;
    state.Vy = odom.twist.twist.linear.y;
    state.omega = odom.twist.twist.angular.z;

    return state;
}

// Save trajectorys to file
EstimatorNode::~EstimatorNode()
{
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EstimatorNode>());
    rclcpp::shutdown();
    return 0;
}

