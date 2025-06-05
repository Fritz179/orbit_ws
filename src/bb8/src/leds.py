#!/usr/bin/env python3
import rospy
import pigpio
import sys

# Configuration
NUM_LEDS = 21
BRIGHTNESS = 15
SPI_CHANNEL = 0  
SPI_BAUD = 4000000 

class DotStarLEDRing:
    def __init__(self):
        # We still initialize rospy for logging and shutdown hook, even without a subscriber
        rospy.init_node('dotstar_led_ring_tester', anonymous=True) # anonymous=True for test nodes

        # Initialize pigpio connection
        self.pi = pigpio.pi()
        if not self.pi.connected:
            rospy.logerr("Couldn't connect to pigpiod. Is the pigpio daemon running?")
            rospy.logerr("Please run 'sudo pigpiod' and ensure it starts correctly.")
            sys.exit(1) # Exit if pigpiod isn't running

        # Open SPI device using pigpio
        # Mode 0 (CPOL=0, CPHA=0) is standard for APA102
        self.spi_handle = self.pi.spi_open(SPI_CHANNEL, SPI_BAUD, 0)
        if self.spi_handle < 0:
            rospy.logerr(f"Failed to open SPI on channel {SPI_CHANNEL} at {SPI_BAUD} baud.")
            rospy.logerr("Ensure SPI is enabled in raspi-config and not in use by another process.")
            self.pi.stop() # Disconnect from pigpiod
            sys.exit(1)

        self.pixels = [(0, 0, 0)] * NUM_LEDS # Internal buffer for LED colors (R, G, B)
        # Combine the global brightness with the fixed 111 prefix for APA102
        # Brightness is a 5-bit value, so it goes from 0-31
        self.brightness_byte = (0b11100000 | (BRIGHTNESS & 0x1F))

        # Register cleanup function to run on ROS node shutdown
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("DotStar LED Ring Tester initialized with pigpio.")

    def set_pixel(self, index, r, g, b):
        """Sets the R, G, B color for a single LED in the internal buffer."""
        if 0 <= index < NUM_LEDS:
            self.pixels[index] = (r, g, b)

    def set_all(self, color):
        """Sets all LEDs to a single color and immediately shows it."""
        r, g, b = color
        for i in range(NUM_LEDS):
            self.pixels[i] = (r, g, b)
        self.show()

    def show(self):
        """Sends the current pixel data from the internal buffer to the LED strip."""
        data = bytearray()
        # Start frame (4 bytes of 0x00)
        data.extend([0x00, 0x00, 0x00, 0x00])

        # LED data for each pixel
        for r, g, b in self.pixels:
            data.append(self.brightness_byte)
            data.append(b & 0xFF)
            data.append(g & 0xFF)
            data.append(r & 0xFF)

        # Correct End frame (at least NUM_LEDS / 2 bits = NUM_LEDS // 16 bytes of 0xFF)
        end_frame_bytes = (NUM_LEDS + 15) // 16
        data.extend([0xFF] * end_frame_bytes)

        # Send data over SPI
        bytes_sent = self.pi.spi_write(self.spi_handle, data)
        if bytes_sent < 0:
            rospy.logwarn(f"SPI write failed with error code: {bytes_sent}")


    def test_pattern_rainbow_chase(self, cycles=3):
        """Runs a chasing rainbow pattern."""
        rospy.loginfo("Starting Rainbow Chase Test Pattern.")
        colors = [
            (255, 0, 0),    # Red
            (255, 127, 0),  # Orange
            (255, 255, 0),  # Yellow
            (0, 255, 0),    # Green
            (0, 0, 255),    # Blue
            (75, 0, 130),   # Indigo
            (148, 0, 211)   # Violet
        ]
        num_colors = len(colors)

        for _ in range(cycles):
            for start_pixel in range(NUM_LEDS):
                self.set_all((0,0,0)) # Clear previous state
                for i in range(NUM_LEDS):
                    color_index = (i + start_pixel) % num_colors
                    self.set_pixel(i, *colors[color_index])
                self.show()
                rospy.sleep(0.05) # Fast chase

    def test_pattern_all_colors(self):
        """Cycles through solid primary and secondary colors."""
        rospy.loginfo("Starting All Colors Test Pattern.")
        test_colors = {
            "Red": (255, 0, 0),
            "Green": (0, 255, 0),
            "Blue": (0, 0, 255),
            "Cyan": (0, 255, 255),
            "Magenta": (255, 0, 255),
            "Yellow": (255, 255, 0),
            "White": (255, 255, 255),
            "Off": (0, 0, 0)
        }
        for name, color in test_colors.items():
            rospy.loginfo(f"Setting all LEDs to {name}")
            self.set_all(color)
            rospy.sleep(1.0)

    def test_pattern_blink_all(self, times=3):
        """Flashes all LEDs white a few times."""
        rospy.loginfo(f"Starting Blink All Test Pattern ({times} times).")
        for _ in range(times):
            self.set_all((255, 255, 255)) # White
            rospy.sleep(0.5)
            self.set_all((0, 0, 0)) # Off
            rospy.sleep(0.5)
        self.set_all((0,0,0)) # Ensure off after blinking

    def cleanup(self):
        """Turns off LEDs and closes pigpio connection on node shutdown."""
        rospy.loginfo("Shutting down DotStar LED Ring Tester.")
        try:
            if self.pi is not None and self.pi.connected:
                self.set_all((0, 0, 0))  # Ensure LEDs are off
                if self.spi_handle >= 0:
                    self.pi.spi_close(self.spi_handle)
                self.pi.stop()
                rospy.loginfo("LEDs off and pigpio connection closed.")
            else:
                rospy.logwarn("pigpio connection was already closed or unavailable.")
        except Exception as e:
            rospy.logerr(f"Error during cleanup: {e}")


if __name__ == "__main__":
    try:
        led_ring = DotStarLEDRing()

        # --- Test Patterns ---
        rospy.loginfo("Running LED test patterns...")

        # Test 1: Cycle through solid colors
        led_ring.test_pattern_all_colors()
        rospy.sleep(1)

        # Test 2: Blink all LEDs
        led_ring.test_pattern_blink_all(times=5)
        rospy.sleep(1)

        # Test 3: Rainbow Chase
        led_ring.test_pattern_rainbow_chase(cycles=2)
        rospy.sleep(1)

        rospy.loginfo("All test patterns complete. LEDs will now turn off.")
        led_ring.set_all((0, 0, 0)) # Ensure all LEDs are off at the end

        # Keep the node running briefly if needed for ROS logging to catch up
        # or remove if you want it to exit immediately after patterns.
        rospy.sleep(2)

    except rospy.ROSInterruptException:
        # Catches Ctrl+C or ROS shutdown signals
        rospy.loginfo("Test interrupted by ROS shutdown.")
        pass
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred during testing: {e}")
    finally:
        # The cleanup method registered with rospy.on_shutdown will handle the pigpio disconnect
        pass