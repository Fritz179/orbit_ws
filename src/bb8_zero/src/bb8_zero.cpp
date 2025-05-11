#include "ros/ros.h"
#include "bb8_zero.h"
#include <algorithm>

NodeZero* node;
const int MOTOR_SPEED = 500;

NodeZero::NodeZero() : m_speed_left(0), m_speed_right(0), 
    
    // DRV8871(    in1_pin, in2_pin);
    m_left_driver( 20,      21),
    m_right_driver(19,      26),

    // ATD5833(    step_pin, dir_pin, enable_pin, ms1_pin, ms2_pin) ls_left, ls_right, max_steps
    m_head(ATD5833(7,        8,       25,         24,      23),     15,      14,       555)

{
    // m_base_state_pub = nh.advertise<std_msgs::Float32>("base_state", 10);
    m_cmd_vel_sub = nh.subscribe("cmd_vel", 10, &NodeZero::cmd_vel_callback, this);
    m_cmd_head_sub = nh.subscribe("cmd_head", 10, &NodeZero::cmd_head_callback, this);

    m_cmd_head_calibrate_sub = nh.subscribe("cmd_head_calibrate", 10, &NodeZero::cmd_head_calibrate_callback, this);

    node = this;

    cmd_head_calibrate_callback_impl();

    ros::Timer timer = nh.createTimer(ros::Duration(0.02),   // 50 Hz
    [&](const ros::TimerEvent&){  
        update_head();
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

    m_left_driver.setSpeed(m_speed_left);
    m_right_driver.setSpeed(m_speed_right);

    print_state();
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

    gpioSetISRFunc(left, RISING_EDGE, 0, NULL);
    gpioSetISRFunc(right, RISING_EDGE, 0, NULL);

    // Go left
    while (!gpioRead(left)) {
        m_head.stepper_driver.step_sync(-1, MOTOR_SPEED);
    }

    m_head.stepper_driver.setMicrostepMode(ATD5833::MicrostepMode::SIXTEENTH);
    int count = 0;

    // Go right and count
    while (!gpioRead(right)) {
        m_head.stepper_driver.step_sync(1, MOTOR_SPEED);
        count++;
    }

    // Exit limit zone
    while (gpioRead(right)) {
        m_head.stepper_driver.step_sync(-1, MOTOR_SPEED);
    }

    ROS_INFO("Motor step count: %d, please update setting", count);
    m_head.state = Head::State::HOAMING;
    m_head.max_steps = count;

    gpioSetISRFunc(left, RISING_EDGE, 0, limit_switch_callback);
    gpioSetISRFunc(right, RISING_EDGE, 0, limit_switch_callback);
    
    print_state();
}

void NodeZero::update_head() {
    if (m_head.state == Head::State::CALIBRATING) return;
    if (m_head.stepper_driver.is_stepping()) return;

    int diff = m_head.desired_steps - m_head.current_steps;
    if (!diff) return;

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

void limit_switch_callback(int pin, int edge, uint32_t tick) {
    node->limit_switch_callback_impl(pin, edge, tick);
}