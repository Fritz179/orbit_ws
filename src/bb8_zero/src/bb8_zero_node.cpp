#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <sstream>

class NodeZero
{
public:
    NodeZero() : m_speed_left(0.0), m_speed_right(0) {
        // m_base_state_pub = nh.advertise<std_msgs::Float32>("base_state", 10);
        m_cmd_vel_sub = nh.subscribe("cmd_vel", 10, &NodeZero::speedCallback, this);
    }

    void spin() {
        ros::Rate loop_rate(2);

        while (ros::ok()) {
            ROS_INFO("Left: '%f', right: '%f'", m_speed_left, m_speed_right);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    void speedCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        m_speed_left = msg->angular.z;
        m_speed_right = msg->angular.z;
        
        ROS_INFO("Set speed to: %f, %f", m_speed_left, m_speed_right);
    }

    float m_speed_right;
    float m_speed_left;

    ros::NodeHandle nh;
    // ros::Publisher  m_base_state_pub;
    ros::Subscriber m_cmd_vel_sub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_zero");
    NodeZero node;
    node.spin();
    return 0;
}
