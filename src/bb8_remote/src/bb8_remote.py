#!/usr/bin/env python3

import pynput
from pynput.keyboard import Key

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseWithCovarianceStamped

from cartographer_ros_msgs.srv import GetTrajectoryStates, FinishTrajectory, StartTrajectory
from cartographer_ros_msgs.msg import TrajectoryStates
from cartographer_ros_msgs.srv import GetTrajectoryStatesRequest, FinishTrajectoryRequest, StartTrajectoryRequest
import rospkg

class Remote:
    def __init__(self):
        self._listener = pynput.keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )

        self._listener.start()
        self._command = [0.0, 0.0, 0.0]
        self._head = 0
        self._enable_head = False
        self._enable = False
        self._auto_enabled = False

        f = rospy.get_param('~forward_rate', 4.0)
        r = rospy.get_param('~rotation_rate', 0.1)
        h = rospy.get_param('~rotation_rate', 0.01)

        self.directions = {
            Key.up: (-1, 0, 0),
            Key.down: (1, 0, 0),
            Key.left: (0, 1, 0),
            Key.right: (0, -1, 0),
            "w": (-1, 0, 0),
            "s": (1, 0, 0),
            "a": (0, 1, 0),
            "d": (0, -1, 0),
            "q": (0, 0, 1),
            "e": (0, 0, -1)
        }

        self.speeds = [f, r, h]

        self._cmd_auto_sub = rospy.Subscriber("/head/cmd_vel", Twist, self.auto)
        self._cmd_pub = rospy.Publisher("/remote/cmd_vel", Twist, queue_size=10)
        self._cmd_head_pub = rospy.Publisher("/remote/cmd_head", Int32, queue_size=10)
        self._enable_pub = rospy.Publisher("/remote/enable", Bool, queue_size=10)
        self._enable_head_pub = rospy.Publisher("/remote/enable_head", Bool, queue_size=10)

        self._pose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.pose_callback)

        # Wait for the services
        rospy.wait_for_service('/head/get_trajectory_states')
        rospy.wait_for_service('/head/finish_trajectory')
        rospy.wait_for_service('/head/start_trajectory')

        # Create service proxies
        self.get_states = rospy.ServiceProxy('/head/get_trajectory_states', GetTrajectoryStates)
        self.finish_trajectory = rospy.ServiceProxy('/head/finish_trajectory', FinishTrajectory)
        self.start_trajectory = rospy.ServiceProxy('/head/start_trajectory', StartTrajectory)

        rospack = rospkg.RosPack()
        self.config_dir = '/home/ros/bb8_ws/src/bb8/config'
        self.config_basename = 'ros_2d_navigation.lua'

    def pose_callback(self, msg):
        rospy.loginfo("Received /initialPose message")

        # Step 1: Get current trajectory states
        try:
            resp = self.get_states()
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call get_trajectory_states service: %s" % e)
            return

        # Step 2: Finish trajectories with state == 0 (active)
        print(resp)
        for i in range(len(resp.trajectory_states.trajectory_id)):
            if resp.trajectory_states.trajectory_state[i] == TrajectoryStates.ACTIVE:
                try:
                    finish_req = FinishTrajectoryRequest()
                    finish_req.trajectory_id = resp.trajectory_states.trajectory_id[i]
                    self.finish_trajectory(finish_req)
                    rospy.loginfo(f"Finished trajectory {i} with state 0")
                except rospy.ServiceException as e:
                    rospy.logerr(f"Failed to finish trajectory {i}: {e}")

        # Step 3: Start a new trajectory with the pose from the message
        try:
            start_req = StartTrajectoryRequest()
            start_req.configuration_directory = self.config_dir
            start_req.configuration_basename = self.config_basename
            start_req.use_initial_pose = True
            start_req.initial_pose  = msg.pose.pose

            start_req.relative_to_trajectory_id = 0  # usually zero unless relative to another trajectory

            self.start_trajectory(start_req)
            rospy.loginfo("Started new trajectory with initial pose")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to start trajectory: {e}")


    def auto(self, data):
        if self._auto_enabled:
            self._command[0] = data.linear.x
            self._command[1] = data.angular.z
            self._command[2] = data.angular.y

        print(data)

    def publish(self):
        msg = Twist()
        msg.linear.x = self._command[0]
        msg.angular.z = self._command[1]
        msg.angular.y = self._command[2]
        self._cmd_pub.publish(msg)

        enable_msg = Bool()
        enable_msg.data = self._enable
        self._enable_pub.publish(enable_msg)

        enable_head_msg = Bool()
        enable_head_msg.data = self._enable_head
        self._enable_head_pub.publish(enable_head_msg)

    def on_press(self, key):
        if hasattr(key, 'char'):
            key = key.char.lower()

        if key == Key.space:
            self._enable = False
        elif key == "b":
            self._enable = True
        elif key == "n":
            self._enable_head = False
        elif key == "m":
            self._enable_head = True
        elif key == "+":
            head_msg = Int32()
            head_msg.data = 10
            self._cmd_head_pub.publish(head_msg)
            self._head += 10
        elif key == "-":
            head_msg = Int32()
            head_msg.data = -10
            self._cmd_head_pub.publish(head_msg)
            self._head -= 10
        elif key == "o":
            self.speeds[0] *= 1.1
        elif key == "p":
            self.speeds[0] /= 1.1
        elif key == "k":
            self.speeds[1] *= 1.1
        elif key == "l":
            self.speeds[1] /= 1.1
        elif key == ".":
            self._auto_enabled = False
        elif key == ",":
            self._auto_enabled = True


        if key in self.directions:
            cmd = self.directions[key]
            for i in [0, 1, 2]:
                if cmd[i] != 0:
                    self._command[i] = cmd[i] * self.speeds[i]

    def on_release(self, key):
        if hasattr(key, 'char'):
            key = key.char.lower()

        if key in self.directions:
            cmd = self.directions[key]
            for i in [0, 1, 2]:
                if cmd[i] != 0:
                    self._command[i] = 0

    def __str__(self):
        return f'''
Commands:
  - Forward:        W | UP
  - Backward:       S | DOWN
  - Left:           A | LEFT
  - Right:          D | RIGHT
  - Head Tilt:      Q | E
  - Head Manual:    + | -

  - Speed:          o | p -> {self.speeds[0]:.3f}
  - rotation:       k | l -> {self.speeds[1]:.3f}

  - Disbale:        SPACE
  - Enable:         B
  - Head Disable:   N
  - Head Enable:    M
  - Autonomous      , | .
        
Satus:
  - Enabled: {self._enable}
  - Head Enabled: {self._enable_head}
  - Forward: {self._command[0]}
  - Rotate: {self._command[1]}
  - Head Rotation: {self._command[2]}
  - Head Manual: {self._head}
  - Autonomous: {self._auto_enabled}
'''


def main():
    rospy.init_node("bb8_teleop")
    teleop = Remote()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        teleop.publish()
        rate.sleep()
        # print(teleop)
        teleop._head = 0

    return 0


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass