"""Continuous motion, so there is something to watch while the rest is built.

Sweeps the arm through a slow repeating pose cycle. This is a placeholder for
the visual servo loop, not part of the story: it exists so the Gazebo window
shows a moving arm rather than a static one.

    ros2 run blindspot_arm wave
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
HOME = [0.0, -1.2, 1.4, -1.75, -1.57, 0.0]


class Wave(Node):

    def __init__(self):
        super().__init__("arm_wave")
        self.pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.t = 0.0
        self.create_timer(0.5, self.tick)

    def tick(self):
        self.t += 0.5
        p = list(HOME)
        p[0] += 0.5 * math.sin(self.t * 0.35)          # pan sweep
        p[1] += 0.25 * math.sin(self.t * 0.25 + 1.0)   # shoulder
        p[2] += 0.30 * math.sin(self.t * 0.30)         # elbow
        p[4] += 0.30 * math.sin(self.t * 0.20)         # wrist tilt
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = p
        pt.time_from_start.sec = 1
        msg.points.append(pt)
        self.pub.publish(msg)


def main(argv=None):
    rclpy.init(args=argv)
    n = Wave()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
