#include "estimator.hpp"

// Random number generator for noise
std::default_random_engine EstimatorNode::generator(std::random_device{}());
std::normal_distribution<double> EstimatorNode::dist(0.0, 1.0);

// Constructor
EstimatorNode::EstimatorNode() : Node("estimator")
{
    m_actuator_feedback_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "/actuator_feedback", 10, std::bind(&EstimatorNode::actuatorFeedbackCallback, this, std::placeholders::_1));
    
    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/sim_ground_truth", 10, std::bind(&EstimatorNode::odomCallback, this, std::placeholders::_1));

    m_estimate_pub = this->create_publisher<nav_msgs::msg::Odometry>("/estimate_odom", 10);

    m_timer = this->create_wall_timer(
        std::chrono::duration<double>(dt), std::bind(&EstimatorNode::timerCallback, this));


    ekf.x << 0, 0, 0; // Initial state with no uncertainty
    ekf.P = Matrix3d::Identity() * 0.1; // Initial covariance matrix with some uncertainty
    ekf.Q = Matrix2d::Identity() * 0.1; // Process noise covariance
    ekf.R = Matrix3d::Identity() * 0.05; // Measurement noise covariance

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
    Vector2d u = m_actuator_feedback;
    Vector3d z = m_ground_truth;

    predict(ekf, u, dt, ekf.Q);
    update(ekf, z, ekf.R);
    publishOdom(ekf.x);

}

// Gets the state vector from the sim and adds a random walk noise
Eigen::Vector3d EstimatorNode::getStateVector(nav_msgs::msg::Odometry odom)
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
    Eigen::LLT<Eigen::Matrix3d> lltOfR(ekf.R);
    Eigen::Matrix3d L = lltOfR.matrixL();
    Eigen::Vector3d noiseScaled = L * noise;

    // Add the scaled noise to the state to simulate measurement noise (random walk)
    state += noiseScaled;

    return state;
}

// Gets the input vector from the sim with no added noise
// There will be uncertainty that comes from whether these values are acheived
Eigen::Vector2d EstimatorNode::getInputVector(sensor_msgs::msg::JointState joint_state)
{
    double a = joint_state.velocity[0] + joint_state.velocity[1] * 0.5; // average angular velocity of back wheels in rad/sec
    double phi = joint_state.position[5]; // front steer position in radians

    Eigen::Vector2d input;
    input(0) = a * r;
    input(1) = (a * r * tan(phi)) / L;

    return input;
}

// Publishes an odom msg with the current estimate
void EstimatorNode::publishOdom(Eigen::Vector3d state)
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

// Normalizes an angle to be within [-pi, pi]
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

// Predicts the next state using the nonlinear model
Vector3d EstimatorNode::predictState(const Vector3d &x, const Vector2d &u, double dt)
{
    double v = u(0);
    double w = u(1);
    double theta = x(2);

    Vector3d x_pred;
    x_pred(0) = x(0) + v * cos(theta) * dt;
    x_pred(1) = x(1) + v * sin(theta) * dt;
    // x_pred(2) = normalizeAngle(x(2) + w * dt);
    x_pred(2) = x(2) + w * dt;
    return x_pred;
}

// Jacobian of the process model with respect to the state (F)
Matrix3d EstimatorNode::computeJacobianF(const Vector3d &x, const Vector2d &u, double dt)
{
    double v = u(0);
    double theta = x(2);

    Matrix3d F;
    F << 1, 0, -v * sin(theta) * dt,
        0, 1, v * cos(theta) * dt,
        0, 0, 1;
    return F;
}

// Jacobian of the process model with respect to the input (B)
Matrix<double, 3, 2> EstimatorNode::computeJacobianB(const Vector3d &x, const Vector2d &u, double dt)
{
    double theta = x(2);

    Matrix<double, 3, 2> B;
    B << cos(theta) * dt, 0,
        sin(theta) * dt, 0,
        0, dt;
    return B;
}

// Prediction step of the EKF
void EstimatorNode::predict(EKF &ekf, const Vector2d &u, double dt, const Matrix2d &Q)
{
    // Predict the new state using the nonlinear model
    Vector3d x_pred = predictState(ekf.x, u, dt);

    // Compute Jacobians
    Matrix3d F = computeJacobianF(ekf.x, u, dt);
    Matrix<double, 3, 2> B = computeJacobianB(ekf.x, u, dt);

    // Predict the covariance matrix
    Matrix3d P_pred = F * ekf.P * F.transpose() + B * Q * B.transpose();

    // Update the state and covariance
    ekf.x = x_pred;
    ekf.P = P_pred;
}

// Update step of the EKF using the measurement z
void EstimatorNode::update(EKF &ekf, const Vector3d &z, const Matrix3d &R)
{
    // Measurement model: h(x) = x, so H is the identity matrix
    Matrix3d H = Matrix3d::Identity();

    // Innovation or residual
    Vector3d y = z - ekf.x;

    // Innovation covariance
    Matrix3d S = H * ekf.P * H.transpose() + R;

    // Kalman gain
    Matrix3d K = ekf.P * H.transpose() * S.inverse();

    // Update the state estimate and covariance
    ekf.x = ekf.x + K * y;
    //ekf.x(2) = normalizeAngle(ekf.x(2));
    ekf.P = (Matrix3d::Identity() - K * H) * ekf.P;
}

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EstimatorNode>());
    rclcpp::shutdown();
    return 0;
}