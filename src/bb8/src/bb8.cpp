#include "bb8.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>  

BB8::BB8() {
    m_pid_pitch_pub = nh.advertise<std_msgs::Float64>("pid_pitch/state", 10);
    m_head_imu_sub = nh.subscribe("/imu_head", 10, &BB8::head_imu_callback, this);

    ROS_INFO("BB8 Node Initialized");
}

void BB8::head_imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    tf2::Quaternion tf_quat;
    tf2::fromMsg(msg->orientation, tf_quat);

    tf2::Matrix3x3 rotation_matrix(tf_quat);

    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);

    std_msgs::Float64 head_pitch;
    head_pitch.data = roll; // Use roll as the head tilt angle
    
    m_pid_pitch_pub.publish(head_pitch);

}
