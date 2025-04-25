#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "bb8_node");

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 10);
    ros::Rate rate(1);

    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = "hello there";
        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}