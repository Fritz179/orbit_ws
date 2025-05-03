#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <tb6612fng/tb6612fng.h>

#include <sstream>

class NodeZero
{
public:

    NodeZero() : m_speed_left(0), m_speed_right(0), 

    // TB6612FNG(stby_pin, pwma, ain1, ain2, pwmb, bin1, bin2);
    m_dc_drv(    14,       19,   5,    6,    13,   0,    11) {
        // m_base_state_pub = nh.advertise<std_msgs::Float32>("base_state", 10);
        m_cmd_vel_sub = nh.subscribe("cmd_vel", 10, &NodeZero::speedCallback, this);
        m_dc_drv.activate();
    }

    void spin() {
        ros::Rate loop_rate(2);

        while (ros::ok()) {
            ROS_INFO("Left: '%d', right: '%d'", m_speed_left, m_speed_right);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    void speedCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        m_speed_left = (int16_t)(msg->linear.x * 255.0 / 5.0);
        m_speed_right = (int16_t)(msg->linear.y * 255.0 / 5.0);

        m_dc_drv.A().setSpeed(m_speed_left);
        m_dc_drv.B().setSpeed(m_speed_right);
        
        ROS_INFO("Set speed to: %f, %f", m_speed_left, m_speed_right);
    }

    int16_t m_speed_right;
    int16_t m_speed_left;

    ros::NodeHandle nh;
    // ros::Publisher  m_base_state_pub;
    ros::Subscriber m_cmd_vel_sub;

    TB6612FNG m_dc_drv;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_zero");
    NodeZero node;
    node.spin();
    return 0;
}