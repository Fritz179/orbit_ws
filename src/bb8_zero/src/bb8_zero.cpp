#include "ros/ros.h"
#include "bb8_zero.h"
#include <algorithm>
#include <cmath>

NodeZero* node;
const int MOTOR_SPEED = 500000;

NodeZero::NodeZero(int pi) : m_speed_left(0), m_speed_right(0), m_PI(pi), 
    // Create PID controllers for each motor (tune gains as needed)
    pid_left(1.0, 0.0, 0.0),
    pid_right(1.0, 0.0, 0.0),    

    // L298N(          in1_pin, in2_pin);
    m_left_driver( pi, 20,      21),
    m_right_driver(pi, 19,      26),

    m_ls_left_cb_id(0),
    m_ls_right_cb_id(0),

    // ATD5833(            step_pin, dir_pin, ms1_pin, ms2_pin) ls_left, ls_right, max_steps
    m_head(pi, ATD5833(pi, 8,        7,       24,      23),     15,      14,       555)

{
    // m_base_state_pub = nh.advertise<std_msgs::Float32>("base_state", 10);
    m_cmd_vel_sub = nh.subscribe("cmd_vel", 10, &NodeZero::cmd_vel_callback, this);
    m_cmd_head_sub = nh.subscribe("cmd_head", 10, &NodeZero::cmd_head_callback, this);
    m_accel_filtered_sub = nh.subscribe("accel/filtered", 10, &NodeZero::accel_filtered_callback, this);
    m_odometry_filtered_sub = nh.subscribe("odometry/filtered", 10, &NodeZero::odometry_filtered_callback, this);
    m_cmd_head_calibrate_sub = nh.subscribe("cmd_head_calibrate", 10, &NodeZero::cmd_head_calibrate_callback, this);

    node = this;

    cmd_head_calibrate_callback_impl();

    ros::Timer timer = nh.createTimer(ros::Duration(0.02),   // 50 Hz
    [&](const ros::TimerEvent&){  
        update_head();
    });

    nh.createTimer(ros::Duration(0.02),   // 50 Hz
    [&](const ros::TimerEvent&){  
        // update_PID();
    });
}

void NodeZero::print_state() {
    if (m_head.state == Head::State::CALIBRATING) {
        ROS_INFO("Motor speed l: %d, r: %d. Head calibrating...", m_speed_left, m_speed_right);
    } else {
        ROS_INFO("Motor speed l: %d, r: %d. Head step current: %d, desired: %d.",
            m_speed_left, m_speed_right, m_head.current_steps, m_head.desired_steps);
    }
}

void NodeZero::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    m_speed_left = (int16_t)(msg->linear.x * 255.0 / 5.0);
    m_speed_right = (int16_t)(msg->linear.y * 255.0 / 5.0);

    ROS_INFO("Setting speed to: left: %d, right: %d", m_speed_left, m_speed_right);

    m_left_driver.setSpeed(m_speed_left);
    m_right_driver.setSpeed(m_speed_right);

    print_state();
    return;
}

void NodeZero::cmd_head_callback(const std_msgs::Float32::ConstPtr& msg) {
    float t = (msg->data + 1) / 2;
    if (t < 0) t = 0;
    if (t > 1) t = 1;

    m_head.desired_steps = (int)(t * (float)m_head.max_steps);

    print_state();
}

void NodeZero::cmd_head_calibrate_callback(const std_msgs::Empty::ConstPtr& msg) {
    cmd_head_calibrate_callback_impl();
}

void NodeZero::cmd_head_calibrate_callback_impl() {
    ROS_INFO("Calibration started...");
    m_head.state = Head::State::CALIBRATING;

    int left = m_head.ls_left;
    int right = m_head.ls_right;

    callback_cancel(m_ls_left_cb_id);
    callback_cancel(m_ls_right_cb_id);
    ROS_INFO("ISRF Removed");

    // Go left
    while (!gpio_read(m_PI, left)) {
        ROS_INFO("Stepping left, %d", gpio_read(m_PI, left));

        m_head.stepper_driver.setDirection(false);
        m_head.stepper_driver.step_sync(1, MOTOR_SPEED);
    }
    ROS_INFO("Left Reached");


    m_head.stepper_driver.setMicrostepMode(ATD5833::MicrostepMode::SIXTEENTH);
    int count = 0;

    ROS_INFO("Going Right");


    // Go right and count
    while (!gpio_read(m_PI, right)) {
        ROS_INFO("Stepping right, %d", gpio_read(m_PI, right));

        m_head.stepper_driver.setDirection(true);
        m_head.stepper_driver.step_sync(1, MOTOR_SPEED);
        count++;
    }

    ROS_INFO("Gone Right");

    // Exit limit zone
    while (gpio_read(m_PI, right)) {
    ROS_INFO("restepping Right, %d", gpio_read(m_PI, right));

        m_head.stepper_driver.setDirection(false);
        m_head.stepper_driver.step_sync(1, MOTOR_SPEED);
    }

    ROS_INFO("Motor step count: %d, please update setting", count);
    m_head.state = Head::State::HOAMING;
    m_head.max_steps = count;

    callback(m_PI, left, RISING_EDGE, limit_switch_callback);
    callback(m_PI, right, RISING_EDGE, limit_switch_callback);
    
    print_state();
}

void NodeZero::update_head() {
    if (m_head.state == Head::State::CALIBRATING) return;
    if (m_head.stepper_driver.is_stepping()) return;

    int diff = m_head.desired_steps - m_head.current_steps;
    if (!diff) return;

    m_head.stepper_driver.setDirection(diff > 0);
    if (diff < 0) diff = -diff;

    m_head.stepper_driver.step_async(diff / m_head.stepper_driver.getMicrostepSize(), MOTOR_SPEED);
    m_head.current_steps += diff;
    print_state();
}

void NodeZero::limit_switch_callback_impl(int pin, int edge, uint32_t tick) {
    bool left = pin == m_head.ls_left;

    ROS_ERROR("%s LIMIT SWITCH ACTIVATED", left ? "LEFT" : "RIGHT");

    m_head.current_steps = left ? 0 : m_head.max_steps;
    m_head.desired_steps = left ? 10 : m_head.max_steps - 10;
}

void limit_switch_callback(int pi, uint32_t pin, uint32_t edge, uint32_t tick) {
    node->limit_switch_callback_impl(pin, edge, tick);
}

void NodeZero::odometry_filtered_callback(const nav_msgs::Odometry::ConstPtr& msg){
    m_odom = *msg;
    ROS_INFO("Pose Orinetation x: %f, y: %f, z: %f, w: %f", 
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    ROS_INFO("Pose Position x: %f, y: %f, z: %f", 
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
    );

    
}

void NodeZero::accel_filtered_callback(const geometry_msgs::AccelWithCovarianceStamped::ConstPtr& msg){
    m_accel = *msg;
    ROS_INFO("Accel Angular x: %f, y: %f, z: %f", 
        msg->accel.accel.angular.x,
        msg->accel.accel.angular.y,
        msg->accel.accel.angular.z);

    ROS_INFO("Accel Linear x: %f, y: %f, z: %f", 
        msg->accel.accel.linear.x,
        msg->accel.accel.linear.y,
        msg->accel.accel.linear.z);

    double xl = msg->accel.accel.linear.x;
    double yl = msg->accel.accel.linear.y;
    double zl = msg->accel.accel.linear.z;
    double xa = msg->accel.accel.angular.x; 
    double ya = msg->accel.accel.angular.y;
    double za = msg->accel.accel.angular.z;

    double absolut_velocity_linear = std::sqrt(xl*xl + yl *yl);
    ROS_INFO("Absolute velocity: %f", absolut_velocity_linear);
    
}

void NodeZero::update_PID(){
    // Estimate forward velocity magnitude (assumes forward is along x)
    double forward_velocity = std::sqrt(
        m_accel.accel.accel.linear.x * m_accel.accel.accel.linear.x + 
        m_accel.accel.accel.linear.y * m_accel.accel.accel.linear.y
    );

    // Basic split for left/right current velocity
    // Positive angular motion: left wheel slower, right wheel faster
    double angular_component = m_accel.accel.accel.angular.z;

    // Estimate individual wheel velocities from linear and angular components
    double wheel_base = 0.2;  // Distance between wheels in meters (adjust!)
    double current_velocity_left = forward_velocity - (angular_component * wheel_base / 2.0);
    double current_velocity_right = forward_velocity + (angular_component * wheel_base / 2.0);

    // Get PID outputs
    double control_left = pid_left.update(m_speed_left, current_velocity_left);
    double control_right = pid_right.update(m_speed_right, current_velocity_right);

    // // Optional: clamp outputs to valid motor range
    // control_left = std::clamp(control_left, -1.0, 1.0);
    // control_right = std::clamp(control_right, -1.0, 1.0);
    
    // m_left_driver.setSpeed(control_left);
    // m_right_driver.setSpeed(control_right);

    print_state();
}

double PID::update(double target, double current) {
    ros::Time now = ros::Time::now();
    double dt = (now - prev_time_).toSec();
    if (dt <= 0.0) dt = 1e-3;

    double error = target - current;
    integral_ += error * dt;
    double derivative = (error - prev_error_) / dt;

    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    prev_error_ = error;
    prev_time_ = now;

    return output;
}