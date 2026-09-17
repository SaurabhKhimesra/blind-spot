"""Save every frame of the film and wrist cameras, named by SIM timestamp.

The clip is assembled offline from these frames and the fold node's log, so
every pixel of robot motion in it was rendered by Gazebo during the run.
"""

import os
import threading
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

TOPICS = {"cine_left": "/cine_left",
          "cine_right": "/cine_right", "wrist_left": "/left/camera/image",
          "wrist_right": "/right/camera/image"}


class Recorder(Node):
    def __init__(self):
        super().__init__("blindspot_record", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.declare_parameter("out", "/tmp/blindspot_frames")
        self.out = self.get_parameter("out").value
        self.pool = ThreadPoolExecutor(max_workers=6)
        self.counts = {k: 0 for k in TOPICS}
        self.lock = threading.Lock()
        qos = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
        for name, topic in TOPICS.items():
            os.makedirs(os.path.join(self.out, name), exist_ok=True)
            self.create_subscription(
                Image, topic, lambda m, n=name: self.on_img(n, m), qos)
        self.create_timer(5.0, self.report)

    def on_img(self, name, m):
        stamp = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        path = os.path.join(self.out, name, "%015d.jpg" % stamp)
        self.pool.submit(cv2.imwrite, path,
                         cv2.cvtColor(img, cv2.COLOR_RGB2BGR),
                         [cv2.IMWRITE_JPEG_QUALITY, 94])
        with self.lock:
            self.counts[name] += 1

    def report(self):
        self.get_logger().info("frames: %s" % self.counts)


def main(argv=None):
    rclpy.init(args=argv)
    n = Recorder()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.pool.shutdown(wait=True)
        n.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
