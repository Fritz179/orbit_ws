#include "bb8.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_bb8");

    ROS_INFO("BB8 Node asdfasdf");

    BB8 node;

    ros::spin();

    return 0;
}