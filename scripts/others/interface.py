#!/usr/bin/env python3
from sshkeyboard import listen_keyboard, stop_listening
from tvmc import MotionController, DoF, ControlMode
import blessings
import rclpy
from rclpy.node import Node
from time import sleep
from threading import Thread
from std_msgs.msg import Float32MultiArray, Float32, Int32MultiArray
from geometry_msgs.msg import Vector3

rclpy.init()

# DATA_SOURCE
DATA_SOURCE = "sensors"

# Constants converted to floats
HEAVE_TARGET_OFFSET = -0.08
HEAVE_KP = -25.0 
HEAVE_KI = 0.0
HEAVE_KD = 60.0
HEAVE_TARGET = 0.35 - HEAVE_TARGET_OFFSET
HEAVE_ACCEPTABLE_ERROR = 0.05
HEAVE_OFFSET = 0.0

PITCH_TARGET_OFFSET = -5.0
PITCH_KP = -0.25
PITCH_KI = 0.0
PITCH_KD = 50.0
PITCH_TARGET = 0.0 - PITCH_TARGET_OFFSET
PITCH_ACCEPTABLE_ERROR = 1.0
PITCH_OFFSET = 0.0 # 5.0

ROLL_KP = 0.0
ROLL_KI = 0.0
ROLL_KD = 0.0
ROLL_TARGET = 3.0
ROLL_ACCEPTABLE_ERROR = 1.0

YAW_TARGET_OFFSET = -7.3
YAW_KP = -2.0
YAW_KI = 0.02
YAW_KD = 12.0
YAW_TARGET = 161.0 - YAW_TARGET_OFFSET
YAW_ACCEPTABLE_ERROR = 0.05

m = MotionController()
term = blessings.Terminal()

closed_loop_enabled = set()
currently_doing = set()
diagnostics = {}
keep_rendering = True


def render():
    while keep_rendering:
        with term.hidden_cursor():
            print(term.fullscreen())
            print(term.clear())
            print(term.red(term.bold("TVMC Controller")))
            print("Press esc. at any point to exit.\n")
            x = 1000

            while keep_rendering and x:
                with term.location(0, 4):
                    print(term.bold("Diagnostics"))

                    for diagnostic in diagnostics:
                        print(
                            f"{diagnostic}:", diagnostics[diagnostic], term.clear_eol()
                        )

                    print(term.clear_eol())
                    print(term.bold("Control"), term.clear_eol())

                    if len(closed_loop_enabled):
                        print(
                            "PID: ",
                            [x.name for x in closed_loop_enabled],
                            term.clear_eol(),
                        )

                    print(
                        "Manual Thrust: ",
                        [x.name for x in currently_doing],
                        term.clear_eol(),
                    )

                    print(term.clear_eol())
                x = x - 1
                sleep(0.01)
    print(term.exit_fullscreen())


def thrust(dof, rev=1.0):
    def p():
        if dof in closed_loop_enabled:
            return

        m.set_thrust(dof, 50.0 * rev)
        currently_doing.add(dof)

    def r():
        if dof in closed_loop_enabled:
            return

        m.set_thrust(dof, 0.0)
        currently_doing.remove(dof)

    return p, r


def pid_enable(dof):
    m.set_control_mode(dof, ControlMode.CLOSED_LOOP)
    closed_loop_enabled.add(dof)

    if dof == DoF.HEAVE:
        m.set_pid_constants(
            DoF.HEAVE,
            HEAVE_KP,
            HEAVE_KI,
            HEAVE_KD,
            HEAVE_ACCEPTABLE_ERROR,
            HEAVE_OFFSET,
        )
        m.set_pid_limits(DoF.HEAVE, -10.0, 10.0, -25.0, 25.0)
        m.set_target_point(DoF.HEAVE, HEAVE_TARGET)

    if dof == DoF.PITCH:
        m.set_pid_constants(
            DoF.PITCH, PITCH_KP, PITCH_KI, PITCH_KD, PITCH_ACCEPTABLE_ERROR
        )
        m.set_pid_limits(DoF.PITCH, -10.0, 10.0, -25.0, 25.0)
        m.set_target_point(DoF.PITCH, PITCH_TARGET)

    if dof == DoF.ROLL:
        m.set_pid_constants(DoF.ROLL, ROLL_KP, ROLL_KI, ROLL_KD, ROLL_ACCEPTABLE_ERROR)
        m.set_target_point(DoF.ROLL, ROLL_TARGET)

    if dof == DoF.YAW:
        m.set_pid_constants(DoF.YAW, YAW_KP, YAW_KI, YAW_KD, YAW_ACCEPTABLE_ERROR)
        m.set_pid_limits(DoF.YAW, -10.0, 10.0, -25.0, 25.0)
        m.set_target_point(DoF.YAW, YAW_TARGET)


def pid_disable(dof):
    m.set_control_mode(dof, ControlMode.OPEN_LOOP)
    closed_loop_enabled.remove(dof)


def pid(dof):
    def toggle():
        if dof in closed_loop_enabled:
            pid_disable(dof)
        else:
            pid_enable(dof)

    return [toggle, lambda: None]


mp = {
    "w": thrust(DoF.SURGE, 3.0),
    "a": thrust(DoF.YAW, -1.0),
    "s": thrust(DoF.SURGE, -1.0),
    "d": thrust(DoF.YAW, 1.0),
    "z": thrust(DoF.HEAVE, 1.0),
    "x": thrust(DoF.HEAVE, -1.0),
    "q": thrust(DoF.ROLL, 1.0),
    "e": thrust(DoF.ROLL, -1.0),
    "u": thrust(DoF.PITCH, 1.0),
    "i": thrust(DoF.PITCH, -1.0),
    "h": pid(DoF.HEAVE),
    "j": pid(DoF.PITCH),
    "k": pid(DoF.ROLL),
    "l": pid(DoF.YAW),
}


def press(key):
    if key in mp:
        mp[key][0]()


def release(key):
    if key in mp:
        mp[key][1]()


def data(node):
    def set_diag(name, data_val):
        diagnostics[name] = data_val

    # subscribe to any topics that you'd like to subscribe to,
    # and then make them update diagnostics_data to have stuff update
    # in real time

    # you should also set up subcribers to
    # update the current point for all PID controllers here

    node.create_subscription(
        Float32MultiArray,
        "/pecka_tvmc/thrust",
        lambda x: set_diag("Thrust", x.data),
        10
    )

    # Changed Int32MultiArray to Float32MultiArray as requested
    node.create_subscription(
        Int32MultiArray, 
        "/control/pwm", 
        lambda x: set_diag("PWM", x.data), 
        10
    )

    node.create_subscription(
        Vector3,
        f"/{DATA_SOURCE}/linear_acceleration",
        lambda x: set_diag("Linear Acceleration", (x.x, x.y, x.z)),
        10
    )

    node.create_subscription(
        Vector3,
        f"/{DATA_SOURCE}/angular_velocity",
        lambda x: set_diag("Angular Velocity", (x.x, x.y, x.z)),
        10
    )

    node.create_subscription(
        Vector3,
        f"/{DATA_SOURCE}/magnetic_field",
        lambda x: set_diag("Magnetic Field", (x.x, x.y, x.z)),
        10
    )

    def orientation(x):
        m.set_current_point(DoF.ROLL, x.x)
        m.set_current_point(DoF.PITCH, x.y)
        m.set_current_point(DoF.YAW, x.z)
        set_diag("Roll", x.x)
        set_diag("Pitch", x.y)
        set_diag("Yaw", x.z)
        
    node.create_subscription(
        Vector3, 
        f"/{DATA_SOURCE}/orientation", 
        orientation, 
        10
    )

    def depth(d):
        m.set_current_point(DoF.HEAVE, d.data)
        set_diag("Depth", d.data)

    node.create_subscription(
        Float32, 
        f"/{DATA_SOURCE}/depth", 
        depth, 
        10
    )


if __name__ == "__main__":

    print(term.red("Starting nodes.\n\n"))
    
    m.start_nodes()
    node = Node("PECKA")
    # Pass the node object to data() to create subscriptions
    data(node)

    # ROS 2 Spin in a background thread
    ros_spinner = Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_spinner.start()

    renderer = Thread(target=render, daemon=True)
    renderer.start()

    listen_keyboard(
        on_press=press,
        on_release=release,
    )

    keep_rendering = False
    print(term.clear())
    print("Bye-bye!\n\n")
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    exit(0)
