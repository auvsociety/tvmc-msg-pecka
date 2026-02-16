#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import blessings
from sshkeyboard import listen_keyboard, stop_listening

PWM_OFF = 1500
PWM_ON = 1600
NUM_THRUSTERS = 7

class PWMTester(Node):
    def __init__(self):
        super().__init__('pwm_tester')
        self.pub = self.create_publisher(Int32MultiArray, '/control/pwm', 10)
        self.term = blessings.Terminal()

    def test_thruster(self, thruster: int):
        msg = Int32MultiArray()
        data = [PWM_OFF] * NUM_THRUSTERS
        
        # If a valid thruster index is provided, turn it ON
        if 1 <= thruster <= NUM_THRUSTERS:
            data[thruster - 1] = PWM_ON
        
        msg.data = data
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    tester = PWMTester()

    print(tester.term.fullscreen())
    print(tester.term.clear())
    print(f"Press (1-{NUM_THRUSTERS}) to test thruster. Q to exit.\n")

    def on_press(key):
        if key == 'q':
            stop_listening()
        elif key.isdigit() and 1 <= int(key) <= NUM_THRUSTERS:
            print(f"\nTesting thruster {int(key)}.")
            tester.test_thruster(int(key))
        else:
            print("\nStopping thrusters.")
            tester.test_thruster(0)

    def on_release(key):
        tester.test_thruster(0)

    try:
        listen_keyboard(on_press=on_press, on_release=on_release)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        tester.test_thruster(0)
        print(tester.term.clear())
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
