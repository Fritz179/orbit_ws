#include "bb8.h"
#include "party.h"
//extern "C" {
//    #include <ws2811.h>
//}

#define LED_COUNT 21
#define LED_PIN 10
#define DMA 10

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_bb8");
    ros::Time::init();

    //ws2811_t ledstring = {0};
/*
    ledstring.freq = WS2811_TARGET_FREQ;
    ledstring.dmanum = DMA;

    ledstring.channel[0].gpionum = LED_PIN;
    ledstring.channel[0].count = LED_COUNT;
    ledstring.channel[0].invert = 0;
    ledstring.channel[0].brightness = 50;
    ledstring.channel[0].strip_type = WS2811_STRIP_GRB;

    // Channel 1 unused
    ledstring.channel[1].gpionum = 0;
    ledstring.channel[1].count = 0;
    ledstring.channel[1].invert = 0;
    ledstring.channel[1].brightness = 0;
*/
    //ws2811_return_t init = ws2811_init(&ledstring);
    //if (init != WS2811_SUCCESS) {
    //    ROS_ERROR("ws2811_init failed, error: %s", ws2811_get_return_t_str(init));
    //    return 1;
    //}

    // sudo bash -c "source /opt/ros/noetic/setup.bash; source ./devel/setup.bash; rosrun bb8 bb8"

    //BB8 node;
    ROS_INFO("BB8 Node Started!");

    // SimpleButtonDetector detector(node.nh);
    //eye(white);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        loop_rate.sleep();
        standby_noise();
        // eye(white);
    }

    //ws2811_fini(&ledstring);

    return 0;
}