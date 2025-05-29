#include "std_msgs/Int32.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/AccelWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"

#include "l298n.h"
#include "lx16a.h"
#include <pigpiod_if2.h>

#include <sstream>

void limit_switch_callback(int pi, uint32_t pin, uint32_t edge, uint32_t tick);

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

class NodeZero
{
public:
    struct Head {
        LX16A servo_driver;

        uint8_t  ls_left;
        uint8_t  ls_right;
        int32_t  desired_steps;

        Head(int pi, LX16A servo, uint8_t ls_left, uint8_t ls_right)
    :   servo_driver(servo),
        ls_left(ls_left),
        ls_right(ls_right),
        desired_steps(0)
        {
            // configure limit‐switch pins as inputs with pull-ups
            set_mode(pi, ls_left,  PI_INPUT);
            set_pull_up_down(pi, ls_left, PI_PUD_UP);
            set_mode(pi, ls_right, PI_INPUT);
            set_pull_up_down(pi, ls_right, PI_PUD_UP);
        }
    };


    NodeZero(int pi, int handle);
    void spin();

    void limit_switch_callback_impl(int pin, int edge, uint32_t tick);
    void print_state();

private:
    int m_PI;

    int16_t m_speed_right;
    int16_t m_speed_left;
    L298N m_left_driver;
    L298N m_right_driver;

    int m_ls_left_cb_id;
    int m_ls_right_cb_id;

    Head m_head;

    ros::NodeHandle nh;

    // set wheel velocity
    ros::Subscriber m_cmd_vel_sub;
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
    
    // set wheel deisred position, -1 = Full Left, 1 = Full Right
    ros::Subscriber m_cmd_head_sub;
    void cmd_head_callback(const std_msgs::Int32::ConstPtr& msg);
    
    // Subscribe ekf
    ros::Subscriber m_accel_filtered_sub;
    void accel_filtered_callback(const geometry_msgs::AccelWithCovarianceStamped::ConstPtr& msg);
    geometry_msgs::AccelWithCovarianceStamped m_accel;
    
    // Subscibe Odometry
    ros::Subscriber m_odometry_filtered_sub; 
    void odometry_filtered_callback(const nav_msgs::Odometry::ConstPtr& msg);
    nav_msgs::Odometry m_odom; 

    PID pid_left;
    PID pid_right;

    ros::Timer m_head_timer;
    void update_head(const ros::TimerEvent&);
    
    ros::Timer m_pid_timer;
    void update_PID(const ros::TimerEvent&);
};