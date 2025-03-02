#include "ukf.hpp"

std::default_random_engine UKFNode::generator(std::random_device{}());
std::normal_distribution<double> UKFNode::dist(0.0, 1.0);

UKFNode::UKFNode() : Node("ukf")
{

    m_actuator_feedback_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "/actuator_feedback", 10, std::bind(&UKFNode::actuatorFeedbackCallback, this, std::placeholders::_1));

    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/sim_ground_truth", 10, std::bind(&UKFNode::odomCallback, this, std::placeholders::_1));

    m_estimate_pub = this->create_publisher<nav_msgs::msg::Odometry>("/ukf_odom", 10);

    //m_noisy_pub = this->create_publisher<nav_msgs::msg::Odometry>("/noisy_odom", 10);

    m_timer = this->create_wall_timer(
        std::chrono::duration<double>(dt), std::bind(&UKFNode::timerCallback, this));


    ukf.x_ = Vector3d::Zero();
    ukf.P_ = Matrix3d::Identity() * 0.1;
    // Process noise covariance (you may adjust these values)
    ukf.Q_ = Matrix3d::Identity() * 0.005;
    // Measurement noise covariance
    ukf.R_ = Matrix3d::Identity() * 0.1;

    // UKF parameters
    alpha_ = 0.1; // usually a small positive number
    kappa_ = 0;     // secondary scaling parameter, often 0 or 3 - n_x
    beta_ = 2;      // optimal for Gaussian distributions
    lambda_ = alpha_ * alpha_ * (n_x + kappa_) - n_x;
    n_sigma_ = 2 * n_x + 1;

    weights_m_ = VectorXd(n_sigma_);
    weights_c_ = VectorXd(n_sigma_);
    weights_m_(0) = lambda_ / (n_x + lambda_);
    weights_c_(0) = weights_m_(0) + (1 - alpha_ * alpha_ + beta_);

    for (int i = 1; i < n_sigma_; i++)
    {
        weights_m_(i) = 1.0 / (2 * (n_x + lambda_));
        weights_c_(i) = weights_m_(i);
    }
}

void UKFNode::actuatorFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state)
{
    m_actuator_feedback = getInputVector(*joint_state);
}

// Update ground truth
void UKFNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    m_ground_truth = getStateVector(*odom);
}

// Gets the state vector from the sim and adds a random walk noise
Eigen::Vector3d UKFNode::getStateVector(nav_msgs::msg::Odometry odom)
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
    Eigen::LLT<Eigen::Matrix3d> lltOfR(ukf.R_);
    Eigen::Matrix3d L = lltOfR.matrixL();
    Eigen::Vector3d noiseScaled = L * noise;

    // Add the scaled noise to the state to simulate measurement noise (random walk)
    state += noiseScaled;

    // Publish the noisy state
    //publishNoisyOdom(state);

    return state;
}

// Gets the input vector from the sim with no added noise
// There will be uncertainty that comes from whether these values are acheived
Eigen::Vector2d UKFNode::getInputVector(sensor_msgs::msg::JointState joint_state)
{
    double a = joint_state.velocity[0] + joint_state.velocity[1] * 0.5; // average angular velocity of back wheels in rad/sec
    double phi = joint_state.position[5];                               // front steer position in radians

    Eigen::Vector2d input;
    input(0) = a * r;
    input(1) = (a * r * tan(phi)) / L;

    return input;
}

// Publishes an odom msg with the current estimate
void UKFNode::publishOdom(Eigen::Vector3d state)
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

// Predict state update at every timer callback
void UKFNode::timerCallback()
{
    Eigen::Vector2d u = m_actuator_feedback;
    Eigen::Vector3d z = m_ground_truth;

    predict(u, dt);
    update(z);
    publishOdom(ukf.x_);
}

double UKFNode::normalizeAngle(double angle)
{
    while (angle > PI)
        angle -= 2 * PI;
    while (angle < -PI)
        angle += 2 * PI;
    return angle;
}

// Process model: predicts the next state given current state x, control input u, and time step dt
Vector3d UKFNode::processModel(const Vector3d &x, const Vector2d &u, double dt)
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

void UKFNode::predict(const Vector2d &u, double dt)
{
    // Generate sigma points from current state and covariance
    MatrixXd sigma_points(n_x, n_sigma_);
    sigma_points.col(0) = ukf.x_;
    Matrix3d A = ((n_x + lambda_) * ukf.P_).llt().matrixL();
    for (int i = 0; i < n_x; i++)
    {
        sigma_points.col(i + 1) = ukf.x_ + A.col(i);
        sigma_points.col(i + 1 + n_x) = ukf.x_ - A.col(i);
    }

    // Propagate each sigma point through the process model
    MatrixXd sigma_points_pred(n_x, n_sigma_);
    for (int i = 0; i < n_sigma_; i++)
    {
        sigma_points_pred.col(i) = processModel(sigma_points.col(i), u, dt);
    }

    // Compute the predicted state mean
    Vector3d x_pred = Vector3d::Zero();
    for (int i = 0; i < n_sigma_; i++)
    {
        x_pred += weights_m_(i) * sigma_points_pred.col(i);
    }
    x_pred(2) = normalizeAngle(x_pred(2)); // normalize the angle

    // Compute the predicted state covariance
    Matrix3d P_pred = Matrix3d::Zero();
    for (int i = 0; i < n_sigma_; i++)
    {
        Vector3d diff = sigma_points_pred.col(i) - x_pred;
        diff(2) = normalizeAngle(diff(2)); // ensure angle difference is normalized
        P_pred += weights_c_(i) * diff * diff.transpose();
    }
    P_pred += ukf.Q_; // add process noise

    // Update the state and covariance
    ukf.x_ = x_pred;
    ukf.P_ = P_pred;
}

void UKFNode::update(const Vector3d &z)
{
    // Regenerate sigma points from predicted state
    MatrixXd sigma_points(n_x, n_sigma_);
    sigma_points.col(0) = ukf.x_;
    Matrix3d A = ((n_x + lambda_) * ukf.P_).llt().matrixL();
    for (int i = 0; i < n_x; i++)
    {
        sigma_points.col(i + 1) = ukf.x_ + A.col(i);
        sigma_points.col(i + 1 + n_x) = ukf.x_ - A.col(i);
    }

    // Transform sigma points into measurement space
    MatrixXd Zsig(n_z, n_sigma_);
    for (int i = 0; i < n_sigma_; i++)
    {
        Zsig.col(i) = sigma_points.col(i);
    }

    // Compute predicted measurement mean
    Vector3d z_pred = Vector3d::Zero();
    for (int i = 0; i < n_sigma_; i++)
    {
        z_pred += weights_m_(i) * Zsig.col(i);
    }
    z_pred(2) = normalizeAngle(z_pred(2));

    // Compute measurement covariance S and cross-correlation matrix Tc
    Matrix3d S = Matrix3d::Zero();
    Matrix3d Tc = Matrix3d::Zero();
    for (int i = 0; i < n_sigma_; i++)
    {
        Vector3d z_diff = Zsig.col(i) - z_pred;
        z_diff(2) = normalizeAngle(z_diff(2));
        S += weights_c_(i) * z_diff * z_diff.transpose();

        Vector3d x_diff = sigma_points.col(i) - ukf.x_;
        x_diff(2) = normalizeAngle(x_diff(2));
        Tc += weights_c_(i) * x_diff * z_diff.transpose();
    }
    S += ukf.R_; // add measurement noise covariance

    // Kalman gain
    Matrix3d K = Tc * S.inverse();

    // Measurement residual
    Vector3d z_diff = z - z_pred;
    z_diff(2) = normalizeAngle(z_diff(2));

    // Update state and covariance
    ukf.x_ = ukf.x_ + K * z_diff;
    ukf.x_(2) = normalizeAngle(ukf.x_(2));
    ukf.P_ = ukf.P_ - K * S * K.transpose();
}

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UKFNode>());
    rclcpp::shutdown();
    return 0;
}