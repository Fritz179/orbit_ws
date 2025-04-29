#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include <sstream>

class NodeZero
{
public:
    NodeZero() : m_speed(9.0f) {
        m_base_state_pub = nh.advertise<std_msgs::Float32>("base_state", 10);
        m_set_speed_sub = nh.subscribe("set_speed", 10, &NodeZero::speedCallback, this);
    }

    void spin() {
        ros::Rate loop_rate(10);

        while (ros::ok()) {
            std_msgs::Float32 msg;
            msg.data = m_speed;
            m_base_state_pub.publish(msg);
            ROS_INFO("Sending: %f", msg.data);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    void speedCallback(const std_msgs::Float32::ConstPtr& msg) {
        m_speed = msg->data;
        
        std_msgs::Float32 ack;
        ack.data = 420.69f;
        m_base_state_pub.publish(ack);
        ROS_INFO("Set speed to: %f", m_speed);
    }

    float m_speed;
    ros::NodeHandle nh;
    ros::Publisher  m_base_state_pub;
    ros::Subscriber m_set_speed_sub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_zero");
    NodeZero node;
    node.spin();
    return 0;
}
