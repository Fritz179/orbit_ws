#include "geometry_msgs/Twist.h"

#include "tb6612fng.h"
#include "a4988.h"

#include <sstream>

class NodeZero
{
public:

    NodeZero();
    void spin();

private:
    void speedCallback(const geometry_msgs::Twist::ConstPtr& msg);

    int16_t m_speed_right;
    int16_t m_speed_left;

    ros::NodeHandle nh;
    // ros::Publisher  m_base_state_pub;
    ros::Subscriber m_cmd_vel_sub;

    TB6612FNG m_dc_driver;
    A4988 m_stepper_driver;

    static bool test;
};