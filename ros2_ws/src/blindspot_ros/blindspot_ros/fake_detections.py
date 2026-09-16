"""Synthetic detections, so the guard node can be exercised without a camera.

Walks a virtual camera through three regimes and publishes the resulting
marker centres, which is enough to see the guard change its mind:

    healthy    all 6 markers on the ring          -> partition ok
    occluded   only 2 markers survive             -> partition dropped
    collapsed  all 6 markers, target near a line  -> partition dropped

    ros2 run blindspot_ros fake_detections
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import (BoundingBox2D, Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)

WIDTH, HEIGHT, FOVY = 1920, 1440, 45.0
PHASE = 40                      # frames per regime


def ring(r=0.05, n=6):
    a = np.linspace(0, 2 * np.pi, n + 1)[:-1]
    return np.stack([r * np.cos(a), r * np.sin(a), np.zeros(n)], axis=1)


def project(points, cTo, fx, fy, cx, cy):
    P = np.hstack([points, np.ones((len(points), 1))])
    P_c = (cTo @ P.T).T[:, :3]
    s = np.stack([P_c[:, 0] / P_c[:, 2], P_c[:, 1] / P_c[:, 2]], axis=1)
    return s * np.array([fx, fy]) + np.array([cx, cy])


class Fake(Node):

    def __init__(self):
        super().__init__("fake_detections")
        fy = (HEIGHT / 2.0) / np.tan(np.deg2rad(FOVY) / 2.0)
        self.K = (fy, fy, (WIDTH - 1) / 2.0, (HEIGHT - 1) / 2.0)
        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.info_pub = self.create_publisher(CameraInfo, "camera_info",
                                              latched)
        self.det_pub = self.create_publisher(Detection2DArray, "detections",
                                             10)
        self.k = 0
        self.publish_info()
        self.create_timer(0.1, self.tick)

    def publish_info(self):
        m = CameraInfo()
        m.width, m.height = WIDTH, HEIGHT
        fx, fy, cx, cy = self.K
        m.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        m.header.frame_id = "camera"
        self.info_pub.publish(m)

    def tick(self):
        phase = (self.k // PHASE) % 3
        pts = ring()
        keep = list(range(6))
        if phase == 1:
            keep = [0, 3]
        elif phase == 2:
            pts = ring()
            pts[:, 1] *= 0.02
        cTo = np.eye(4)
        cTo[:3, 3] = [0.02 * np.sin(self.k * 0.05), -0.01, 0.62]
        uv = project(pts, cTo, *self.K)

        msg = Detection2DArray()
        msg.header.frame_id = "camera"
        msg.header.stamp = self.get_clock().now().to_msg()
        # deliberately shuffled: a detector reports whatever order it finds,
        # and the guard node must recover the order from the ids.
        order = list(keep)
        rng = np.random.default_rng(self.k)
        rng.shuffle(order)
        for i in order:
            d = Detection2D()
            d.id = str(i)
            b = BoundingBox2D()
            b.center.position.x = float(uv[i, 0])
            b.center.position.y = float(uv[i, 1])
            b.size_x, b.size_y = 40.0, 40.0
            d.bbox = b
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = str(i)
            h.hypothesis.score = 1.0
            d.results.append(h)
            msg.detections.append(d)
        self.det_pub.publish(msg)

        if self.k % PHASE == 0:
            self.get_logger().info(
                "phase %d: %s" % (phase, ["healthy, 6 markers",
                                          "occluded, 2 markers",
                                          "collapsed target, 6 markers"][phase]))
        self.k += 1


def main(argv=None):
    rclpy.init(args=argv)
    n = Fake()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
