import rclpy
from rclpy.node import Node
from threading import Thread
from time import sleep
import subprocess
import signal

from .enums import DoF, Command, ControlMode
import pecka_tvmc_msg.msg as msg


PREFIX = "/pecka_tvmc/control"
PACKAGE = "pecka_tvmc"
TVMC_EXECUTABLE = "tvmc"
PWMC_EXECUTABLE = "pwmc"


class MotionController(Node):
    def __init__(self):
        super().__init__("motion_controller")

        # init control modes
        self.controlModes = dict.fromkeys(DoF, ControlMode.OPEN_LOOP)

        # create publishers
        self._thruster_pub = self.create_publisher(msg.Thrust, f"{PREFIX}/thrust", 50)
        self._command_pub = self.create_publisher(msg.Command, f"{PREFIX}/command", 50)
        self._control_mode_pub = self.create_publisher(msg.ControlMode, f"{PREFIX}/control_mode", 50)
        self._current_state_pub = self.create_publisher(msg.CurrentPoint, f"{PREFIX}/current_point", 50)
        self._pid_constants_pub = self.create_publisher(msg.PidConstants, f"{PREFIX}/pid_constants", 50)
        self._pid_limits_pub = self.create_publisher(msg.PidLimits, f"{PREFIX}/pid_limits", 50)
        self._target_state_pub = self.create_publisher(msg.TargetPoint, f"{PREFIX}/target_point", 50)

        # subprocess handles for external nodes
        self._tvmc_process = None
        self._pwmc_process = None

        # thread for spinning (like rospy.spin in background)
        self._spinner = Thread(target=rclpy.spin, args=(self,), daemon=True)
        self._spinner.start()

    # ----------------- Node launching -----------------
    def start_nodes(self):
        """Start tvmc and pwmc like roslaunch did in ROS1"""
        self.get_logger().info("Starting TVMC and PWMC nodes...")

        self._tvmc_process = subprocess.Popen(
            ["ros2", "run", PACKAGE, TVMC_EXECUTABLE],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self._pwmc_process = subprocess.Popen(
            ["ros2", "run", PACKAGE, PWMC_EXECUTABLE],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        sleep(1)  # give them time to start
        self.get_logger().info("TVMC and PWMC started.")

    def shutdown(self):
        """Graceful shutdown: stop child processes and cleanup"""
        self.get_logger().info("Shutting down MotionController...")

        # send shutdown command to tvmc
        shutdown_msg = msg.Command()
        shutdown_msg.command = Command.SHUT_DOWN.value
        self._command_pub.publish(shutdown_msg)

        sleep(0.5)

        # kill child processes if running
        for proc in [self._tvmc_process, self._pwmc_process]:
            if proc and proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                proc.wait(timeout=2)

        self.destroy_node()
        self.get_logger().info("MotionController stopped.")

    # ----------------- Interface methods -----------------
    def set_thrust(self, dof: DoF, thrust: float) -> None:
        if self.controlModes[dof] == ControlMode.CLOSED_LOOP:
            raise AssertionError(f"Cannot set thrust for DoF {dof} in closed loop mode.")

        t = msg.Thrust()
        t.dof = dof.value
        t.thrust = thrust
        self._thruster_pub.publish(t)

    def set_control_mode(self, dof: DoF, control: ControlMode) -> None:
        mode = msg.ControlMode()
        mode.dof = dof.value
        mode.mode = control.value
        self._control_mode_pub.publish(mode)
        self.controlModes[dof] = control

    def send_command(self, command: Command) -> None:
        command_msg = msg.Command()
        command_msg.command = command.value
        self._command_pub.publish(command_msg)

    def set_current_point(self, dof: DoF, current_point: float):
        point = msg.CurrentPoint()
        point.dof = dof.value
        point.current = current_point
        self._current_state_pub.publish(point)

    def set_pid_constants(self, dof: DoF, kp, ki, kd, acceptable_error, ko=0.0):
        pid_msg = msg.PidConstants()
        pid_msg.dof = dof.value
        pid_msg.kp = kp
        pid_msg.ki = ki
        pid_msg.kd = kd
        pid_msg.acceptable_error = acceptable_error
        pid_msg.ko = ko
        self._pid_constants_pub.publish(pid_msg)

    def set_pid_limits(self, dof: DoF, integral_min, integral_max, output_min, output_max):
        limits = msg.PidLimits()
        limits.dof = dof.value
        limits.integral_max = integral_max
        limits.integral_min = integral_min
        limits.output_max = output_max
        limits.output_min = output_min
        self._pid_limits_pub.publish(limits)

    def set_target_point(self, dof: DoF, target: float) -> None:
        point = msg.TargetPoint()
        point.dof = dof.value
        point.target = target
        self._target_state_pub.publish(point)


def main():
    rclpy.init()
    controller = MotionController()
    controller.start_nodes()   # start tvmc + pwmc

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

