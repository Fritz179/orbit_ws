#include "std_msgs/Float32.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/AccelWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"

#include "drv8871.h"
#include "atd5833.h"

#include <sstream>

void limit_switch_callback(int pin, int edge, uint32_t tick);

class NodeZero
{
public:
    struct Head {
        enum State {
            // Go to both limit switches to calibrate position and step range
            CALIBRATING,

            // try to make current_steps = desired_steps
            HOAMING,
        };

        ATD5833 stepper_driver;

        uint8_t  ls_left;
        uint8_t  ls_right;
        uint32_t max_steps;
        uint32_t current_steps;
        uint32_t desired_steps;
        State    state;

        Head(ATD5833 stepper, uint8_t ls_left, uint8_t ls_right, uint32_t max_steps)
    :   stepper_driver(stepper),
        ls_left(ls_left),
        ls_right(ls_right),
        max_steps(max_steps),
        current_steps(max_steps / 2),
        desired_steps(max_steps / 2),
        state(State::CALIBRATING)
        {
            // configure limit‐switch pins as inputs with pull-ups
            gpioSetMode(ls_left,  PI_INPUT);
            gpioSetPullUpDown(ls_left, PI_PUD_UP);
            gpioSetMode(ls_right, PI_INPUT);
            gpioSetPullUpDown(ls_right, PI_PUD_UP);
        }
    };


    NodeZero();
    void spin();

    void limit_switch_callback_impl(int pin, int edge, uint32_t tick);
    void print_state();

private:
    int16_t m_speed_right;
    int16_t m_speed_left;
    DRV8871 m_left_driver;
    DRV8871 m_right_driver;

    Head m_head;

    ros::NodeHandle nh;

    // set wheel velocity
    ros::Subscriber m_cmd_vel_sub;
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
    
    // set wheel deisred position, -1 = Full Left, 1 = Full Right
    ros::Subscriber m_cmd_head_sub;
    void cmd_head_callback(const std_msgs::Float32::ConstPtr& msg);
    
    // Start head calibration
    ros::Subscriber m_cmd_head_calibrate_sub;
    void cmd_head_calibrate_callback(const std_msgs::Empty::ConstPtr& msg);
    void cmd_head_calibrate_callback_impl();

    // Subscribe ekf
    ros::Subscriber m_accel_filtered_sub;
    void accel_filtered_callback(const geometry_msgs::AccelWithCovarianceStamped::ConstPtr& msg);
    geometry_msgs::AccelWithCovarianceStamped m_accel;
    
    // Subscibe Odometry
    ros::Subscriber m_odometry_filtered_sub; 
    void odometry_filtered_callback(const nav_msgs::Odometry::ConstPtr& msg);
    nav_msgs::Odometry m_odom; 

    void update_head();
    void update_PID();
};

class PID {
    public:
        PID(double kp, double ki, double kd)
            : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0), prev_time_(ros::Time::now()) {}
    
        double update(double target, double current);
    private:
        double kp_, ki_, kd_;
        double prev_error_, integral_;
        ros::Time prev_time_;
    };