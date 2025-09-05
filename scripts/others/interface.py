#!/usr/bin/env python3
# ROS 2 port of the original ROS 1 interface.py with same functionality.

from sshkeyboard import listen_keyboard, stop_listening
from tvmc import MotionController, DoF, ControlMode  # MotionController is ROS 2-based
import blessings
import rclpy
from time import sleep
from threading import Thread
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Float32
from geometry_msgs.msg import Vector3

# -------------------- Config --------------------

DATA_SOURCE = "sensors"

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
PITCH_OFFSET = 0.0

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

# -------------------- Globals --------------------

m = None  # will be set to MotionController in main()
term = blessings.Terminal()

closed_loop_enabled = set()
currently_doing = set()
diagnostics = {}
keep_rendering = True

# -------------------- UI / Render --------------------

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
                        print(f"{diagnostic}:", diagnostics[diagnostic], term.clear_eol())

                    print(term.clear_eol())
                    print(term.bold("Control"), term.clear_eol())

                    if len(closed_loop_enabled):
                        print("PID: ", [x.name for x in closed_loop_enabled], term.clear_eol())

                    print("Manual Thrust: ", [x.name for x in currently_doing], term.clear_eol())

                    print(term.clear_eol())
                x = x - 1
                sleep(0.01)
    print(term.exit_fullscreen())

# -------------------- Controls --------------------

def thrust(dof, rev=1.0):
    def p():
        if dof in closed_loop_enabled:
            return
        m.set_thrust(dof, float(50.0 * rev))
        currently_doing.add(dof)

    def r():
        if dof in closed_loop_enabled:
            return
        m.set_thrust(dof, 0.0)
        currently_doing.discard(dof)

    return p, r

def pid_enable(dof):
    m.set_control_mode(dof, ControlMode.CLOSED_LOOP)
    closed_loop_enabled.add(dof)

    if dof == DoF.HEAVE:
        m.set_pid_constants(
            DoF.HEAVE, HEAVE_KP, HEAVE_KI, HEAVE_KD, HEAVE_ACCEPTABLE_ERROR, HEAVE_OFFSET
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
        m.set_pid_constants(
            DoF.ROLL, ROLL_KP, ROLL_KI, ROLL_KD, ROLL_ACCEPTABLE_ERROR
        )
        m.set_target_point(DoF.ROLL, ROLL_TARGET)

    if dof == DoF.YAW:
        m.set_pid_constants(
            DoF.YAW, YAW_KP, YAW_KI, YAW_KD, YAW_ACCEPTABLE_ERROR
        )
        m.set_pid_limits(DoF.YAW, -10.0, 10.0, -25.0, 25.0)
        m.set_target_point(DoF.YAW, YAW_TARGET)

def pid_disable(dof):
    m.set_control_mode(dof, ControlMode.OPEN_LOOP)
    closed_loop_enabled.discard(dof)

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

# -------------------- Subscriptions (ROS 2) --------------------

def data():
    """Create subscriptions on the MotionController node (ROS 2)."""
    def set_diag(name, data):
        diagnostics[name] = data

    m.create_subscription(Float32MultiArray, "/pecka_tvmc/thrust",
                          lambda x: set_diag("Thrust", list(x.data)), 10)

    m.create_subscription(Int32MultiArray, "/control/pwm",
                          lambda x: set_diag("PWM", list(x.data)), 10)

    m.create_subscription(Vector3, f"/{DATA_SOURCE}/linear_acceleration",
                          lambda x: set_diag("Linear Acceleration", (float(x.x), float(x.y), float(x.z))), 10)

    m.create_subscription(Vector3, f"/{DATA_SOURCE}/angular_velocity",
                          lambda x: set_diag("Angular Velocity", (float(x.x), float(x.y), float(x.z))), 10)

    m.create_subscription(Vector3, f"/{DATA_SOURCE}/magnetic_field",
                          lambda x: set_diag("Magnetic Field", (float(x.x), float(x.y), float(x.z))), 10)

    def orientation(x: Vector3):
        m.set_current_point(DoF.ROLL, float(x.x))
        m.set_current_point(DoF.PITCH, float(x.y))
        m.set_current_point(DoF.YAW, float(x.z))
        set_diag("Roll", float(x.x))
        set_diag("Pitch", float(x.y))
        set_diag("Yaw", float(x.z))

    m.create_subscription(Vector3, f"/{DATA_SOURCE}/orientation", orientation, 10)

    def depth_cb(d: Float32):
        m.set_current_point(DoF.HEAVE, float(d.data))
        set_diag("Depth", float(d.data))

    m.create_subscription(Float32, f"/{DATA_SOURCE}/depth", depth_cb, 10)

# -------------------- Main --------------------

def main():
    global m, keep_rendering

    rclpy.init()
    m = MotionController()
    print(term.red("Starting nodes.\n\n"))

    if hasattr(m, "start_nodes"):
        m.start_nodes()
    elif hasattr(m, "start"):
        m.start()
    else:
        m.get_logger().warn("No start/start_nodes method found; not auto-launching tvmc/pwmc.")

    renderer = Thread(target=render, daemon=True)
    renderer.start()

    data()

    listen_keyboard(on_press=press, on_release=release)

    keep_rendering = False
    print(term.clear())
    print("Bye-bye!\n\n")

    if hasattr(m, "shutdown"):
        m.shutdown()
    else:
        m.destroy_node()

    rclpy.shutdown()
    exit(0)

if __name__ == "__main__":
    main()

