#!/usr/bin/env python3

import pynput
from pynput.keyboard import Key

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Int32

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

        self._cmd_pub = rospy.Publisher("/remote/cmd_vel", Twist, queue_size=10)
        self._cmd_head_pub = rospy.Publisher("/remote/cmd_head", Int32, queue_size=10)
        self._enable_pub = rospy.Publisher("/remote/enable", Bool, queue_size=10)
        self._enable_head_pub = rospy.Publisher("/remote/enable_head", Bool, queue_size=10)

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
        
Satus:
  - Enabled: {self._enable}
  - Head Enabled: {self._enable_head}
  - Forward: {self._command[0]}
  - Rotate: {self._command[1]}
  - Head Rotation: {self._command[2]}
  - Head Manual: {self._head}
'''


def main():
    rospy.init_node("bb8_teleop")
    teleop = Remote()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        teleop.publish()
        rate.sleep()
        print(teleop)
        teleop._head = 0

    return 0


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


# 4, 0.1, -10