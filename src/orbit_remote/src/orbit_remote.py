#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32, Empty

import spidev, time

SYM0 = 0b1000        # WS ‘0’  → 1000
SYM1 = 0b1110        # WS ‘1’  → 1110

def expand_byte(b: int) -> bytes:
    out = 0
    for _ in range(8):
        out = (out << 4) | (SYM1 if b & 0x80 else SYM0)
        b <<= 1
    return out.to_bytes(4, "big")

def wheel(pos: int):
    if pos < 85:   return pos*3, 255-pos*3, 0
    if pos < 170:  pos -= 85; return 255-pos*3, 0, pos*3
    pos -= 170;    return 0, pos*3, 255-pos*3

class LEDCyclerSPI:
    def __init__(self):
        self.n      = rospy.get_param("~count",   94)
        bus         = rospy.get_param("~bus",      0)
        dev         = rospy.get_param("~device",   0)
        hz          = rospy.get_param("~hz", 2400000)
        self.delay  = rospy.get_param("~delay", 0.02)
        self.step   = rospy.get_param("~step",  10)

        self.spi = spidev.SpiDev(bus, dev)
        self.spi.max_speed_hz = hz

        self.phase = 0
        self.timer = rospy.Timer(rospy.Duration(self.delay), self.update)

    def send(self, colors):
        buf = bytearray()
        for r, g, b in colors:          # WS order = G R B
            for ch in (g, r, b):
                buf += expand_byte(ch)
        self.spi.xfer3(buf)

    def update(self, _):
        frame = [wheel((i*256//self.n + self.phase) & 255)
                 for i in range(self.n)]
        self.send(frame)
        self.phase = (self.phase + self.step) & 255



class Remote:
    def __init__(self):
        # Params
        self.f = rospy.get_param('~forward_rate', 3.5)
        self.r = rospy.get_param('~rotation_rate', 0.5)
        self.h = rospy.get_param('~head_rate', 10)

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
        self._head_manual = 0

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

        if msg.axes[self.BTN_SPEED]:
            self.f += msg.axes[self.BTN_SPEED] / 100

        if msg.axes[self.BTN_ROTATIO_SPEED]:
            self.r += msg.axes[self.BTN_ROTATIO_SPEED] / -500

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

        # Axes -> Twist
        cmd = Twist()
        cmd.linear.x  = msg.axes[self.AXIS_LY] * self.f      # invert up/down if needed
        cmd.angular.z = msg.axes[self.AXIS_LX] * self.r
        print(cmd, self._enable)
        print(f"Speed: {self.f}, Rotation: {self.r}")

        self.pub_cmd.publish(cmd)
        self.pub_enable.publish(Bool(self._enable))
        self.pub_enable_head.publish(Bool(self._enable_head))

def main():
    rospy.init_node("orbit_remote")
    Remote()
    LEDCyclerSPI()
    rospy.spin()

if __name__ == "__main__":
    main()
