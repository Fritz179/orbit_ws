#include "ros/ros.h"
#include "bb8_zero.h"
#include <algorithm>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>  

NodeZero* node;

NodeZero::NodeZero(int pi, int handle) : 
    m_enabled(false), m_head_enabled(false),
    m_odom(), m_accel(),
    m_speed_left(0), m_speed_right(0), 
    m_PI(pi), 

    // L298N(          in1_pin, in2_pin);
    m_left_driver( pi, 20,      21),
    m_right_driver(pi, 19,      26),

    //                               ls_left, ls_right
    m_head(pi, LX16A(pi, handle, 1), 9,       25),

    nh(),
    m_pid_speed(nh, "/sphere/pid_speed", &NodeZero::update_motors, this),
    m_pid_heading(nh, "/sphere/pid_heading", &NodeZero::update_motors, this),
    m_pid_pitch(nh, "/sphere/pid_pitch", &NodeZero::update_pitch, this)
{
    // Limit switch hack
    gpio_write(pi, 10, 0); // ls_left =>  3.3V,      10 = GND, 9 = S
    gpio_write(pi, 24, 1); // ls_right => 24 = 3.3V, GND,      25 = S

    // Enable and Commands
    m_enable_sub = nh.subscribe("/remote/enable", 10, &NodeZero::enable_callback, this);
    m_enable_head_sub = nh.subscribe("/remote/enable_head", 10, &NodeZero::enable_head_callback, this);
    m_cmd_vel_sub = nh.subscribe("/remote/cmd_vel", 10, &NodeZero::cmd_vel_callback, this);
    m_cmd_head_sub = nh.subscribe("/remote/cmd_head", 10, &NodeZero::cmd_head_callback, this);

    // Sphere IMU, EKF, PID
    m_accel_filtered_sub = nh.subscribe("/sphere/accel/filtered", 10, &NodeZero::accel_filtered_callback, this);
    m_odometry_filtered_sub = nh.subscribe("/sphere/odometry/filtered", 10, &NodeZero::odometry_filtered_callback, this);
    

    // Head IMU and PID
    m_head_imu_sub = nh.subscribe("/head/imu/data_100hz", 10, &NodeZero::head_imu_callback, this);

    node = this;

    m_head.servo_driver.set_speed(0);

    double temp = m_head.servo_driver.get_voltage();
    ROS_INFO("Voltage: %f", temp);

    ROS_INFO("ID: %d", m_head.servo_driver.get_ID());
    
    callback(m_PI, m_head.ls_left, RISING_EDGE, limit_switch_callback);
    callback(m_PI, m_head.ls_right, RISING_EDGE, limit_switch_callback);

    m_print_state_timer = nh.createTimer(ros::Duration(0.1), &NodeZero::print_state, this);
}

void NodeZero::print_state(const ros::TimerEvent&) {
    ROS_INFO("En %d, Head %d, Motor speed: %f, l: %d, r: %d. Servo: %d. Yaw %f, Head pitch: %f", 
        m_enabled, m_head_enabled, m_pid_speed.setpoint, m_speed_left, m_speed_right, m_head.desired_steps, m_odom.pose.pose.orientation.z, m_pid_pitch.setpoint);
}

void NodeZero::head_imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    tf2::Quaternion tf_quat;
    tf2::fromMsg(msg->orientation, tf_quat);

    tf2::Matrix3x3 rotation_matrix(tf_quat);

    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);

    m_pid_pitch.pub_state(roll);
}


void NodeZero::enable_callback(const std_msgs::Bool::ConstPtr& msg) {
    m_enabled = msg->data;

    if (!m_enabled) {
        m_speed_left = 0;
        m_speed_right = 0;
        m_left_driver.setSpeed(0);
        m_right_driver.setSpeed(0);

        m_head.servo_driver.set_speed(0);
        m_head.desired_steps = 0;
    }
}

void NodeZero::enable_head_callback(const std_msgs::Bool::ConstPtr& msg) {
    m_head_enabled = msg->data;

    update_pitch();
}

void NodeZero::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {

    m_pid_speed.pub_setpoint(msg->linear.x);
    m_pid_pitch.pub_setpoint(m_pid_pitch.setpoint + msg->angular.y);

    if (msg->angular.z) {
        m_pid_heading.pub_setpoint(std::fmod(m_odom.pose.pose.orientation.z + msg->angular.z + M_PI, 2 * M_PI) - M_PI);
    }

    // ROS_INFO("Got cmd_vel: speed: %d, angle: %f", m_speed, msg->angular.z);
}

void NodeZero::update_motors() {
    if (!m_enabled) {
        m_left_driver.setSpeed(0);
        m_right_driver.setSpeed(0);
        return;
    }

    m_speed_left = (m_pid_speed.setpoint + m_pid_heading.setpoint) * 255.0;
    m_speed_right = (m_pid_speed.setpoint - m_pid_heading.setpoint) * 255.0;

    m_left_driver.setSpeed(m_speed_left);
    m_right_driver.setSpeed(m_speed_right);
}


void NodeZero::cmd_head_callback(const std_msgs::Int32::ConstPtr& msg) {
    m_head.desired_steps = msg->data;
}

void NodeZero::update_pitch() {

    // Everything is disabled
    if (!m_enabled) {
        m_head.servo_driver.set_speed(0);
        return;
    }

    // Only manual steps
    if (m_head.desired_steps) {
        if (m_head.desired_steps > 0) {
            m_head.servo_driver.set_speed(gpio_read(m_PI, m_head.ls_right) ? 0 : 500);
            m_head.desired_steps--;
        } else {
            m_head.servo_driver.set_speed(gpio_read(m_PI, m_head.ls_left) ? 0 : -500);
            m_head.desired_steps++;
        }

        return;
    }

    // Automatic disabled
    if (!m_head_enabled) {
        m_head.servo_driver.set_speed(0);
        return;
    }

    if (m_pid_pitch.effort > 0 && !gpio_read(m_PI, m_head.ls_right)) {
        m_head.servo_driver.set_speed(m_pid_pitch.effort * 1000.0);
    } else if (m_pid_pitch.effort < 0 && !gpio_read(m_PI, m_head.ls_left)) {
        m_head.servo_driver.set_speed(m_pid_pitch.effort * 1000.0);
    } else {
        m_head.servo_driver.set_speed(0);
    }
}

void NodeZero::limit_switch_callback_impl(int pin, int edge, uint32_t tick) {
    m_head.servo_driver.set_speed(0);

    bool left = pin == m_head.ls_left;
    ROS_ERROR("%s LIMIT SWITCH ACTIVATED", left ? "LEFT" : "RIGHT");

    // If the other is not activated
    if (!gpio_read(m_PI, left ? m_head.ls_right : m_head.ls_left)) {
        m_head.desired_steps = left ? 10 : -10;
    }
}

void limit_switch_callback(int pi, uint32_t pin, uint32_t edge, uint32_t tick) {
    node->limit_switch_callback_impl(pin, edge, tick);
}

void NodeZero::odometry_filtered_callback(const nav_msgs::Odometry::ConstPtr& msg){
    m_odom = *msg;

    m_pid_heading.pub_state(msg->pose.pose.orientation.z);
    m_pid_speed.pub_state(msg->twist.twist.linear.x);
    
    // ROS_INFO("Pose Orinetation x: %f, y: %f, z: %f, w: %f", 
    //     msg->pose.pose.orientation.x,
    //     msg->pose.pose.orientation.y,
    //     msg->pose.pose.orientation.z,
    //     msg->pose.pose.orientation.w
    // );

    // ROS_INFO("Pose Position x: %f, y: %f, z: %f", 
    //     msg->pose.pose.position.x,
    //     msg->pose.pose.position.y,
    //     msg->pose.pose.position.z
    // );
}

void NodeZero::accel_filtered_callback(const geometry_msgs::AccelWithCovarianceStamped::ConstPtr& msg){
    m_accel = *msg;
    // ROS_INFO("Accel Angular x: %f, y: %f, z: %f", 
    //     msg->accel.accel.angular.x,
    //     msg->accel.accel.angular.y,
    //     msg->accel.accel.angular.z);

    // ROS_INFO("Accel Linear x: %f, y: %f, z: %f", 
    //     msg->accel.accel.linear.x,
    //     msg->accel.accel.linear.y,
    //     msg->accel.accel.linear.z);

    // double xl = msg->accel.accel.linear.x;
    // double yl = msg->accel.accel.linear.y;
    // double zl = msg->accel.accel.linear.z;
    // double xa = msg->accel.accel.angular.x; 
    // double ya = msg->accel.accel.angular.y;
    // double za = msg->accel.accel.angular.z;

    // double absolut_velocity_linear = std::sqrt(xl*xl + yl *yl);
    // ROS_INFO("Absolute velocity: %f", absolut_velocity_linear);
}