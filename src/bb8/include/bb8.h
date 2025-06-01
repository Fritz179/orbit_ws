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

};