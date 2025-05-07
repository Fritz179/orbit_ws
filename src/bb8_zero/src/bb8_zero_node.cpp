#include "ros/ros.h"
#include "bb8_zero.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_zero");

    if (gpioInitialise() < 0) {
        ROS_ERROR("pigpio init failed");
    }

    NodeZero node;
    ros::spin();

    gpioTerminate();
    return 0;
}