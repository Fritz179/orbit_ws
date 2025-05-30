#include "bb8.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>  

BB8::BB8() {
    m_head_tilt_pub = nh.advertise<std_msgs::Float64>("pid_pitch/state", 10);
    m_head_imu_sub = nh.subscribe("/imu_head", 10, &BB8::head_imu_callback, this);

    ROS_INFO("BB8 Node Initialized");
}

void BB8::head_imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    // Convert ROS message quaternion to tf2 quaternion
    tf2::Quaternion tf_quat;
    tf2::fromMsg(msg->orientation, tf_quat);
    // tf_quat.normalize(); // Optional: ensure unit quaternion, skipped for brevity

    // Create rotation matrix from quaternion
    tf2::Matrix3x3 rotation_matrix(tf_quat);

    // Extract Roll, Pitch, Yaw (in radians)
    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);

    // Create a Float64 message to publish the head tilt
    std_msgs::Float64 head_pitch;
    head_pitch.data = roll; // Use pitch as the head tilt angle
    m_head_tilt_pub.publish(head_pitch);

}
