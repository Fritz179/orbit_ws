#include "ros/ros.h"
#include "bb8_zero.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_zero");

    int pi = pigpio_start(NULL, NULL); 
    if (pi < 0) {
        ROS_ERROR("pigpio init failed");
        return 1;
    }

    NodeZero node(pi);
    ros::spin();

    pigpio_stop(pi);
    return 0;
}