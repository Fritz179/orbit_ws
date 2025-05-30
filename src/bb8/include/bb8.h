#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"

class BB8 {
public:
    BB8();
    void spin();

    ros::NodeHandle nh;


private:
    // Head pitch
    ros::Subscriber m_head_imu_sub;
    ros::Publisher m_pid_pitch_pub;
    void head_imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
};