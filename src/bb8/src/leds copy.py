#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from apa102 import APA102

# Configuration
NUM_LEDS = 21
BRIGHTNESS = 0.5  # 0-31

class DotStarLEDRing:
    def __init__(self):
        rospy.init_node('dotstar_led_ring')
        self.strip = APA102(count=NUM_LEDS, brightness=BRIGHTNESS)
        self.sub = rospy.Subscriber("led_ring_cmd", String, self.callback)
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("DotStar LED Ring Controller initialized.")

    def callback(self, msg):
        command = msg.data.lower()
        if command == "on":
            self.set_all((255, 255, 255))
        elif command == "off":
            self.set_all((0, 0, 0))
        elif command.startswith("blink"):
            try:
                _, count_str = command.split()
                self.blink(int(count_str))
            except:
                rospy.logwarn("Invalid blink command. Use 'blink N'.")
        elif command == "eye":
            self.eye()
        elif command == "siren":
            self.siren()
        else:
            rospy.logwarn(f"Unknown command: {command}")

    def set_all(self, color):
        for i in range(NUM_LEDS):
            self.strip.set_pixel(i, *color)
        self.strip.show()

    def blink(self, times):
        for _ in range(times):
            self.set_all((255, 255, 255))
            rospy.sleep(0.5)
            self.set_all((0, 0, 0))
            rospy.sleep(0.5)

    def eye(self):
        self.set_all((0, 0, 0))
        self.strip.set_pixel(0, 255, 0, 0)
        for i in range(8, NUM_LEDS):
            self.strip.set_pixel(i, 255, 0, 0)
        self.strip.show()
        rospy.loginfo("Eye pattern activated.")

    def siren(self):
        for _ in range(5):
            self.set_all((255, 0, 0))
            rospy.sleep(0.3)
            self.set_all((0, 0, 255))
            rospy.sleep(0.3)
        self.set_all((0, 0, 0))

    def cleanup(self):
        self.set_all((0, 0, 0))
        self.strip.cleanup()
        rospy.loginfo("LEDs off and SPI closed.")

if __name__ == "__main__":
    try:
        DotStarLEDRing()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
