#include "ros/ros.h"
#include "bb8_zero.h"

NodeZero::NodeZero() : m_speed_left(0), m_speed_right(0), 
    // TB6612FNG(stby_pin, pwma, ain1, ain2, pwmb, bin1, bin2);
    m_dc_driver( 14,       19,   5,    6,    13,   0,    11),
    // A4988(        step_pin, dir_pin, enable_pin, ms1_pin, ms2_pin, ms3_pin)
    m_stepper_driver(1,        1,       1,          1,       1,       1) {
        // m_base_state_pub = nh.advertise<std_msgs::Float32>("base_state", 10);
        m_cmd_vel_sub = nh.subscribe("cmd_vel", 10, &NodeZero::speedCallback, this);
        m_dc_driver.activate();
    }

void NodeZero::spin() {
    ros::Rate loop_rate(2);

    while (ros::ok()) {
        ROS_INFO("Left: '%d', right: '%d'", m_speed_left, m_speed_right);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void NodeZero::speedCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    m_speed_left = (int16_t)(msg->linear.x * 255.0 / 5.0);
    m_speed_right = (int16_t)(msg->linear.y * 255.0 / 5.0);

    m_dc_driver.A().setSpeed(m_speed_left);
    m_dc_driver.B().setSpeed(m_speed_right);
    
    ROS_INFO("Set speed to: %d, %d", m_speed_left, m_speed_right);
}