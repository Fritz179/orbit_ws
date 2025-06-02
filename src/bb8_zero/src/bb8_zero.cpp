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
    m_heading(0), m_pitch(0), m_odom(), m_accel(),
    m_speed(0), m_speed_left(0), m_speed_right(0), 
    m_PI(pi), 

    // L298N(          in1_pin, in2_pin);
    m_left_driver( pi, 20,      21),
    m_right_driver(pi, 19,      26),

    //                               ls_left, ls_right
    m_head(pi, LX16A(pi, handle, 1), 9,       25)

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
    m_pid_heading_setpoint_pub = nh.advertise<std_msgs::Float64>("/sphere/pid_heading/setpoint", 10);
    m_pid_heading_state_pub = nh.advertise<std_msgs::Float64>("/sphere/pid_heading/state", 10);
    m_pid_heading_effort_sub = nh.subscribe("/sphere/pid_heading/control_effort", 10, &NodeZero::pid_heading_effort_callback, this);

    // Head IMU and PID
    m_head_imu_sub = nh.subscribe("/head/imu/data_100hz", 10, &NodeZero::head_imu_callback, this);
    m_pid_pitch_pub = nh.advertise<std_msgs::Float64>("/sphere/pid_pitch/state", 10);
    m_pid_pitch_setpoint_pub = nh.advertise<std_msgs::Float64>("/sphere/pid_pitch/setpoint", 10);
    m_pid_pitch_effort_sub = nh.subscribe("/sphere/pid_pitch/control_effort", 10, &NodeZero::pid_pitch_effort_callback, this);

    node = this;

    m_head.servo_driver.set_speed(0);

    double temp = m_head.servo_driver.get_voltage();
    ROS_INFO("Voltage: %f", temp);

    ROS_INFO("ID: %d", m_head.servo_driver.get_ID());
    
    callback(m_PI, m_head.ls_left, RISING_EDGE, limit_switch_callback);
    callback(m_PI, m_head.ls_right, RISING_EDGE, limit_switch_callback);

    m_head_timer = nh.createTimer(ros::Duration(0.02), &NodeZero::update_head, this);
    m_print_state_timer = nh.createTimer(ros::Duration(0.1), &NodeZero::print_state, this);

    // Send the first setpoint.
    ros::Rate loop_rate(1);
    while (m_pid_heading_setpoint_pub.getNumSubscribers() == 0 || m_pid_pitch_setpoint_pub.getNumSubscribers() == 0) {
        ROS_INFO("Waiting for subscribers to %s...", m_pid_heading_setpoint_pub.getTopic().c_str());
        
        if (!ros::ok()) {
            return;
        }

        loop_rate.sleep();
    }

    loop_rate.sleep();

    std_msgs::Float64 setpoint;
    setpoint.data = m_heading;
    m_pid_heading_setpoint_pub.publish(setpoint);
    m_pid_pitch_setpoint_pub.publish(setpoint);
}

void NodeZero::print_state(const ros::TimerEvent&) {
    ROS_INFO("En %d, Head %d, Motor speed: %d, l: %d, r: %d. Servo: %d. Yaw %f, Head pitch vel: %f", 
        m_enabled, m_head_enabled, m_speed, m_speed_left, m_speed_right, m_head.desired_steps, m_odom.pose.pose.orientation.z, m_pitch_vel);
}

void NodeZero::head_imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    tf2::Quaternion tf_quat;
    tf2::fromMsg(msg->orientation, tf_quat);

    tf2::Matrix3x3 rotation_matrix(tf_quat);

    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);

    std_msgs::Float64 head_pitch;
    head_pitch.data = roll; // Use roll as the head tilt angle
    
    m_pid_pitch_pub.publish(head_pitch);

    m_pitch_vel = msg->angular_velocity.x; // Use x as the pitch velocity
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
}

void NodeZero::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    m_speed = (int16_t)(msg->linear.x * 255.0 / 5.0);

    if (msg->angular.z) {
        m_heading = m_odom.pose.pose.orientation.z + msg->angular.z;

        m_heading = std::fmod(m_heading + 1, 2) - 1; // Normalize to [-1, 1]
    }

    m_pitch += msg->angular.y;

    std_msgs::Float64 heading_setpoint;
    heading_setpoint.data = m_heading;
    m_pid_heading_setpoint_pub.publish(heading_setpoint);

    std_msgs::Float64 pitch_setpoint;
    pitch_setpoint.data = m_pitch;
    m_pid_pitch_setpoint_pub.publish(pitch_setpoint);

    // ROS_INFO("Got cmd_vel: speed: %d, angle: %f", m_speed, msg->angular.z);
}

void NodeZero::pid_heading_effort_callback(const std_msgs::Float64::ConstPtr& msg) {
    // ROS_INFO("Heading effort received: %f", msg->data);
    
    if (m_enabled) {
        m_speed_left = m_speed + (msg->data * 255.0);
        m_speed_right = m_speed - (msg->data * 255.0);

        m_left_driver.setSpeed(m_speed_left);
        m_right_driver.setSpeed(m_speed_right);
    }
};

void NodeZero::pid_pitch_effort_callback(const std_msgs::Float64::ConstPtr& msg) {
    // ROS_INFO("Pitching effort received: %f", msg->data);
    
    if (m_enabled && m_head_enabled && m_head.desired_steps == 0) {
        if (m_pitch_vel > 0.8) {
            ROS_INFO("Pitching right");
            m_head.servo_driver.set_speed(gpio_read(m_PI, m_head.ls_right) ? 0 : -1000.0);
            return;
        } else if (m_pitch_vel < -0.8) {
            ROS_INFO("Pitching left");
            m_head.servo_driver.set_speed(gpio_read(m_PI, m_head.ls_left) ? 0 : 1000.0);
            return;
        }   

        if (msg->data > 0 && !gpio_read(m_PI, m_head.ls_right)) {
            m_head.servo_driver.set_speed(msg->data * 1000.0);

        } else if (msg->data < 0 && !gpio_read(m_PI, m_head.ls_left)) {
            m_head.servo_driver.set_speed(msg->data * 1000.0);

        } else {
            m_head.servo_driver.set_speed(0);
        }

    }
};


void NodeZero::cmd_head_callback(const std_msgs::Int32::ConstPtr& msg) {
    m_head.desired_steps = msg->data;
}

void NodeZero::update_head(const ros::TimerEvent&) {
    if (!m_enabled) {
        m_head.servo_driver.set_speed(0);
        return;
    }


    if (!m_head.desired_steps) {
        if (!m_head_enabled) {
            m_head.servo_driver.set_speed(0);
        }

        return;
    }

    if (m_head.desired_steps > 0) {
        m_head.servo_driver.set_speed(gpio_read(m_PI, m_head.ls_right) ? 0 : 500);
        m_head.desired_steps--;
    } else {
        m_head.servo_driver.set_speed(gpio_read(m_PI, m_head.ls_left) ? 0 : -500);
        m_head.desired_steps++;
    }

    // It was the last one
    if (!m_head.desired_steps) {
        m_head.servo_driver.set_speed(0);
        return;
    }
}

void NodeZero::limit_switch_callback_impl(int pin, int edge, uint32_t tick) {
    bool left = pin == m_head.ls_left;

    ROS_ERROR("%s LIMIT SWITCH ACTIVATED", left ? "LEFT" : "RIGHT");
    wave_clear(m_PI);
    wave_tx_stop(m_PI);

    // If the other is also activated
    if (gpio_read(m_PI, left ? m_head.ls_right : m_head.ls_left)) {
        m_head.servo_driver.set_speed(0);
    } else {
        m_head.desired_steps = left ? 10 : -10;
    }
}

void limit_switch_callback(int pi, uint32_t pin, uint32_t edge, uint32_t tick) {
    node->limit_switch_callback_impl(pin, edge, tick);
}

void NodeZero::odometry_filtered_callback(const nav_msgs::Odometry::ConstPtr& msg){
    m_odom = *msg;

    std_msgs::Float64 state;
    state.data = msg->pose.pose.orientation.z;
    m_pid_heading_state_pub.publish(state);
    
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