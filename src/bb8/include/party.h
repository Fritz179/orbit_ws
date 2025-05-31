#ifndef PARTY_H
#define PARTY_H

extern "C" {
    #include <ws2811.h>
}

#include <sensor_msgs/Joy.h>
#include <cstdlib>

ws2811_t ledstring = {0};

void eye(uint8_t color[3]);

// Button indices
const int BUTTON_A_INDEX = 0;
const int BUTTON_B_INDEX = 1;
const int BUTTON_X_INDEX = 2;
const int BUTTON_Y_INDEX = 3;

const int n_beeps = 7;
std::string beeps[n_beeps] = {"tones/beep_1.wav", "tones/beep_2.wav", "tones/beep_3.wav", "tones/beep_4.wav", "tones/beep_5.wav", "tones/beep_6.wav", "tones/beep_7.wav"};

const int n_songs = 2;
std::string songs[n_songs] = {"music/black_in_black.wav", "music/dani_california.wav"};


void system_wrapper(const char* command) {
    int ret = system(command);

    if (ret) {
        ROS_ERROR("Command: %s, returned with exit code: %d", command, ret);
    }
}


void random_sound() { system_wrapper(("aplay -q " + beeps[rand() % n_beeps]).c_str()); }
void random_song() { system_wrapper(("aplay -q " + songs[rand() % n_songs]).c_str()); }
void screem() { system_wrapper("aplay -q tones/r2scream.wav"); }
void standby_noise() { if (rand() % 1000 == 0) random_sound(); }
void stop_sound() { system_wrapper("pkill -f aplay"); }

class SimpleButtonDetector {
public:
    SimpleButtonDetector(ros::NodeHandle& nh_) : nh(nh_) {
        joy_sub_ = nh.subscribe("joy", 10, &SimpleButtonDetector::joyCallback, this);
        button_states_.resize(4, 0);
    }

    bool isButtonPressed(int button_index) const {
        return button_states_[button_index];
    }

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
        if (msg->buttons.size() >= 4) {
            button_states_[BUTTON_A_INDEX] = msg->buttons[BUTTON_A_INDEX];
            button_states_[BUTTON_B_INDEX] = msg->buttons[BUTTON_B_INDEX];
            button_states_[BUTTON_X_INDEX] = msg->buttons[BUTTON_X_INDEX];
            button_states_[BUTTON_Y_INDEX] = msg->buttons[BUTTON_Y_INDEX];
        } else {
            ROS_WARN_THROTTLE(1.0, "Joy message has fewer than 4 buttons.");
        }

        if (button_states_[BUTTON_A_INDEX]) {
            stop_sound(); random_sound();
        } else if (button_states_[BUTTON_B_INDEX]) {
            stop_sound(); random_song();
        } else if (button_states_[BUTTON_X_INDEX]) {
            stop_sound();
        }
    }

    ros::Subscriber joy_sub_;
    std::vector<int> button_states_;
    ros::NodeHandle nh;
};

uint8_t white[3]   = {255, 255, 255};
uint8_t red[3]     = {255, 0, 0};
uint8_t green[3]   = {0, 255, 0};
uint8_t blue[3]    = {0, 0, 255};
uint8_t yellow[3]  = {255, 255, 0};
uint8_t cyan[3]    = {0, 255, 255};
uint8_t magenta[3] = {255, 0, 255};
uint8_t black[3]   = {0, 0, 0};

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <string>
#include <vector>
#include <iostream>
#include <ctime>

#include <party.h>


int loading_counter = 0;
int pos = 0;

// Convert (R,G,B) to packed format
uint32_t colour(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

void setPixelColor(int index, uint32_t c) {
    ledstring.channel[0].leds[index] = c;
}

void show() {
    ws2811_render(&ledstring);
}

void eye(uint8_t color[3]) {
    setPixelColor(0, colour(color[0], color[1], color[2]));
    for (int i = 1; i <= 9; ++i) setPixelColor(i, colour(black[0], black[1], black[2]));
    for (int i = 10; i <= 20; ++i) setPixelColor(i, colour(color[0], color[1], color[2]));
    show();
}

void loading_screen(uint8_t color[3]) {
    if (loading_counter > 10){
        setPixelColor(pos + 9, colour(black[0], black[1], black[2]));
        pos = (pos == 20) ? 9 : pos + 1;
        setPixelColor(pos + 9, colour(color[0], color[1], color[2]));
    } else loading_counter++;
}

#endif // PARTY_H
