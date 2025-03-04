#include "cf.hpp"

// Random number generator for noise
std::default_random_engine CFNode::generator(std::random_device{}());
std::normal_distribution<double> CFNode::dist(0.0, 1.0);

// Constructor
CFNode::CFNode() : Node("cf")
{
    m_actuator_feedback_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "/actuator_feedback", 10, std::bind(&CFNode::actuatorFeedbackCallback, this, std::placeholders::_1));
    
    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/sim_ground_truth", 10, std::bind(&CFNode::odomCallback, this, std::placeholders::_1));

    m_estimate_pub = this->create_publisher<nav_msgs::msg::Odometry>("/cf_odom", 10);

    m_timer = this->create_wall_timer(
        std::chrono::duration<double>(dt), std::bind(&CFNode::timerCallback, this));

    est_state << 0, 0, 0; // Initial state with no uncertainty
}

// Update actuator feedback
void CFNode::actuatorFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state)
{
    m_actuator_feedback = getInputVector(*joint_state);
}

// Update ground truth
void CFNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    m_ground_truth = getStateVector(*odom);
}

// Gets the state vector from the sim and adds a random walk noise
Eigen::Vector3d CFNode::getStateVector(nav_msgs::msg::Odometry odom)
{
    Eigen::Vector3d state;
    state(0) = odom.pose.pose.position.x;
    state(1) = odom.pose.pose.position.y;
    // state(2) = normalizeAngle(tf2::getYaw(odom.pose.pose.orientation));
    state(2) = tf2::getYaw(odom.pose.pose.orientation);

    // Generate a 3x1 vector of independent standard normal random variables.
    Eigen::Vector3d noise;
    for (int i = 0; i < 3; ++i)
    {
        noise(i) = dist(generator);
    }

    // Use Cholesky decomposition to transform the standard normal noise
    // into noise with covariance R. Here, L is a lower triangular matrix such that R = L * L^T.

    Matrix3d R = Matrix3d::Identity() * 0.05; // Measurement noise covariance

    Eigen::LLT<Eigen::Matrix3d> lltOfR(R);
    Eigen::Matrix3d L = lltOfR.matrixL();
    Eigen::Vector3d noiseScaled = L * noise;

    // Add the scaled noise to the state to simulate measurement noise (random walk)
    state += noiseScaled;

    // Publish the noisy state
    // publishNoisyOdom(state);

    return state;
}

// Gets the input vector from the sim with no added noise
// There will be uncertainty that comes from whether these values are acheived
Eigen::Vector2d CFNode::getInputVector(sensor_msgs::msg::JointState joint_state)
{
    double a = (joint_state.velocity[0] + joint_state.velocity[1]) * 0.5; // average angular velocity of back wheels in rad/sec
    double phi = joint_state.position[5];                                 // front steer position in radians

    Eigen::Vector2d input;
    input(0) = a * r;
    input(1) = (a * r * tan(phi)) / L;

    return input;
}


// timer callback
void CFNode::timerCallback()
{
    Vector2d u = m_actuator_feedback;
    Vector3d z = m_ground_truth;

    est_state = processModel(est_state, u, dt);
    est_state(2) = normalizeAngle(est_state(2));

    est_state = (est_state * alpha) + (z * (1 - alpha));

    publishOdom(est_state);
}

// normalize angle
double CFNode::normalizeAngle(double angle)
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
void CFNode::publishOdom(Eigen::Vector3d state)
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


// Process model: predicts the next state given current state x, control input u, and time step dt
Vector3d CFNode::processModel(const Vector3d &x, const Vector2d &u, double dt)
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
    rclcpp::spin(std::make_shared<CFNode>());
    rclcpp::shutdown();
    return 0;
}
