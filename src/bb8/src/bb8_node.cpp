#include "bb8.h"
#include "party.h"

#define LED_COUNT 21
#define LED_PIN 12
#define DMA 10

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_bb8");

    ledstring = {0};

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

    // if (ws2811_init(&ledstring) != WS2811_SUCCESS) {
    //     fprintf(stderr, "ws2811_init failed\n");
    //     return 1;
    // }

    BB8 node;
    ROS_INFO("BB8 Node Started!");

    // srand(time(NULL));
    // SimpleButtonDetector detector(node.nh);
    // eye(white);


    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        // standby_noise();
        // loop_rate.sleep();
    }

    // ws2811_fini(&ledstring);

    return 0;
}