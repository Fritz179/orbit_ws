#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/AccelWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

#include "l298n.h"
#include "lx16a.h"
#include <pigpiod_if2.h>

#include <sstream>

void limit_switch_callback(int pi, uint32_t pin, uint32_t edge, uint32_t tick);

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

    L298N m_left_driver;
    L298N m_right_driver;

    Head m_head;

    ros::NodeHandle nh;

    // Emergency stop
    ros::Subscriber m_enable_sub;
    void enable_callback(const std_msgs::Bool::ConstPtr& msg);
    bool m_enabled;

    // Enable head movement
    ros::Subscriber m_enable_head_sub;
    void enable_head_callback(const std_msgs::Bool::ConstPtr& msg);
    bool m_head_enabled;

    // set wheel velocity
    ros::Subscriber m_cmd_vel_sub;
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
    int16_t m_speed;
    int16_t m_speed_right;
    int16_t m_speed_left;
    
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

    // Heading PID
    ros::Publisher m_pid_heading_setpoint_pub;
    ros::Publisher m_pid_heading_state_pub;
    ros::Subscriber m_pid_heading_effort_sub;
    void pid_heading_effort_callback(const std_msgs::Float64::ConstPtr& msg);
    double m_heading;

    // Pitch PID
    ros::Subscriber m_head_imu_sub;
    ros::Publisher m_pid_pitch_pub;
    void head_imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
    double m_pitch_vel;

    ros::Publisher m_pid_pitch_setpoint_pub;
    ros::Subscriber m_pid_pitch_effort_sub;
    void pid_pitch_effort_callback(const std_msgs::Float64::ConstPtr& msg);
    double m_pitch;

    ros::Timer m_head_timer;
    void update_head(const ros::TimerEvent&);

    ros::Timer m_print_state_timer;
    void print_state(const ros::TimerEvent&);
};