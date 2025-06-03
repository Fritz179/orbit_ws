#ifndef PID_H
#define PID_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <string>
#include <functional>

class PID
{
public:
  PID(const PID&) = delete;
  PID& operator=(const PID&) = delete;

  /**
   * @brief Templated constructor: uses the provided NodeHandle to advertise
   *        "<ns>/state" and "<ns>/setpoint", and subscribe to "<ns>/control_effort".
   *        Whenever a new effort arrives, we store it in m_effort and then invoke (obj->*user_cb)().
   *
   * @tparam T         Your node‐class type (e.g. Node)
   * @param nh         ROS NodeHandle to use for advertising/subscribing
   * @param ns         Base topic namespace, e.g. "/head/pid"
   * @param user_cb    Pointer to your zero‐arg member function (void T::foo())
   * @param obj        Pointer to your node (i.e. "this")
   */
  template <typename T>
  PID(ros::NodeHandle& nh, const std::string& ns, void (T::*user_cb)(), T* obj) 
    : setpoint(0.0),
      state(0.0),
      effort(0.0)
  {
    // Bind your member function (no args) into our std::function
    m_user_cb = std::bind(user_cb, obj);

    m_state_pub = nh.advertise<std_msgs::Float64>(ns + "/state", 10);
    m_setpoint_pub = nh.advertise<std_msgs::Float64>(ns + "/setpoint", 10);
    m_effort_sub = nh.subscribe(ns + "/control_effort", 10, &PID::effort_callback, this);

    // Send the first setpoint.
    ros::Rate loop_rate(1);
    while (m_setpoint_pub.getNumSubscribers() == 0) {
        ROS_INFO("Waiting for subscribers to %s...", m_setpoint_pub.getTopic().c_str());
        
        if (!ros::ok()) {
            return;
        }

        loop_rate.sleep();
    }

    loop_rate.sleep();

    std_msgs::Float64 setpoint;
    setpoint.data = 0.0;
    m_setpoint_pub.publish(setpoint);
  }

  /**
   * @brief Publish the current process variable ("state") on "<ns>/state"
   */
  void pub_state(double value)
  {
    state = value;
    std_msgs::Float64 msg;
    msg.data = state;
    m_state_pub.publish(msg);
  }

  /**
   * @brief Publish the desired setpoint on "<ns>/setpoint"
   *        (also stores it in m_setpoint for your own use).
   */
  void pub_setpoint(double value)
  {
    setpoint = value;
    std_msgs::Float64 msg;
    msg.data = setpoint;
    m_setpoint_pub.publish(msg);
  }

  double setpoint;    ///  last‐published setpoint
  double state;       ///  last‐published state
  double effort;      ///  most recent control effort received

private:
  ros::Publisher  m_state_pub;
  ros::Publisher  m_setpoint_pub;
  ros::Subscriber m_effort_sub;

  // Internally stores "obj->*user_cb" bound to "this" node
  std::function<void()> m_user_cb;

  /**
   * @brief ROS callback for "<ns>/control_effort".  Stores msg->data
   *        into m_effort, then invokes m_user_cb().
   */
  void effort_callback(const std_msgs::Float64::ConstPtr& msg)
  {
    effort = msg->data;
    
    if (m_user_cb)
    {
      m_user_cb();
    }
  }
};

#endif  // PID_H
