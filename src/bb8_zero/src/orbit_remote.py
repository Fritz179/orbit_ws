#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32, Empty

class Remote:
    def __init__(self):
        # Params
        self.f = rospy.get_param('~forward_rate', 0.6)
        self.r = rospy.get_param('~rotation_rate', 0.4)
        self.h = rospy.get_param('~head_rate', 4)

        self.curr_linear = 0.0

        # PS5 mapping (adjust if needed)
        self.AXIS_LX = rospy.get_param('~axis_angular', 0)   # left stick horiz
        self.AXIS_LY = rospy.get_param('~axis_linear', 1)    # left stick vert
        self.AXIS_RX = rospy.get_param('~axis_head', 2)      # right stick horiz? choose what you need
        self.BTN_ENABLE = rospy.get_param('~btn_enable', 4)  # L1
        self.BTN_HEAD_ENABLE = rospy.get_param('~btn_head_enable', 5)  # R1
        self.BTN_HEAD_UP = rospy.get_param('~btn_head_up', 3)   # Triangle
        self.BTN_HEAD_DOWN = rospy.get_param('~btn_head_down', 0) # Cross
        self.BTN_SPEED = rospy.get_param('~btn_speed', 7) # Cross
        self.BTN_ROTATIO_SPEED = rospy.get_param('~btn_rotation_speed', 6) # Cross

        self._enable = False
        self._enable_head = False

        # pubs
        self.pub_cmd = rospy.Publisher("/remote/cmd_vel", Twist, queue_size=10)
        self.pub_cmd_head = rospy.Publisher("/remote/cmd_head", Int32, queue_size=10)
        self.pub_enable = rospy.Publisher("/remote/enable", Bool, queue_size=10)
        self.pub_enable_head = rospy.Publisher("/remote/enable_head", Bool, queue_size=10)
        self.pub_start_sound = rospy.Publisher("/play_start_sound", Empty, queue_size=10)
        self.pub_stop_sound  = rospy.Publisher("/play_stop_sound",  Empty, queue_size=10)

        rospy.Subscriber("/remote/joy", Joy, self.cb, queue_size=10)  # if you remap /joy -> /remote/joy; else just "/joy"

    def cb(self, msg: Joy):
        print(msg)
        # Buttons
        was_enabled = self._enable
        self._enable = bool(msg.buttons[self.BTN_ENABLE])
        self._enable_head = bool(msg.buttons[self.BTN_HEAD_ENABLE])

        # if msg.axes[self.BTN_SPEED]:
        #     self.f += msg.axes[self.BTN_SPEED] / 100

        # if msg.axes[self.BTN_ROTATIO_SPEED]:
        #     self.r += msg.axes[self.BTN_ROTATIO_SPEED] / -500

        if msg.buttons[self.BTN_HEAD_UP]:
            self._head_manual = 10
            self.pub_cmd_head.publish(Int32(10))
            print(10)
        if msg.buttons[self.BTN_HEAD_DOWN]:
            self._head_manual -= 10
            self.pub_cmd_head.publish(Int32(-10))
            print(-10)

        if self._enable and not was_enabled:
            self.pub_start_sound.publish(Empty())
        if not self._enable and was_enabled:
            self.pub_stop_sound.publish(Empty())

        desired = msg.axes[self.AXIS_LY] * self.f
        self.curr_linear = self.curr_linear * 0.92 + desired * 0.08

        # Axes -> Twist
        cmd = Twist()
        cmd.linear.x  = self.curr_linear      # invert up/down if needed
        cmd.angular.z = msg.axes[self.AXIS_LX] * self.r
        # print(cmd, self._enable)
        # print(f"Speed: {self.f}, Rotation: {self.r}")

        self.pub_cmd.publish(cmd)
        self.pub_enable.publish(Bool(self._enable))
        self.pub_enable_head.publish(Bool(self._enable_head))

def main():
    rospy.init_node("orbit_remote")
    Remote()
    rospy.spin()

if __name__ == "__main__":
    main()
