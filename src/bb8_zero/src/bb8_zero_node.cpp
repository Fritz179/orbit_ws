#include "ros/ros.h"
#include "bb8_zero.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_zero");

    int pi = pigpio_start(NULL, NULL); 
    if (pi < 0) {
        ROS_ERROR("pigpio init failed: %d", pi);
        return 1;
    }

    int handle = serial_open(pi, (char*)"/dev/ttyUSB0", 115200, 0);
    if (handle < 0) {
        ROS_ERROR("pigpio handle init failed: %d", handle);
        return 1;
    }

    NodeZero node(pi, handle);
    ROS_INFO("NodeZero create succesfully!");

    ros::spin();

    serial_close(pi, handle);
    pigpio_stop(pi);
    return 0;
}