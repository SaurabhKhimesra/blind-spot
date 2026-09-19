"""Publishes, per detection frame, whether the 2001 partition is safe now.

    ros2 run blindspot_ros guard_node --ros-args \
        -p calibration:=/abs/path/my_target.json \
        -r detections:=/aruco/detections \
        -r camera_info:=/camera/camera_info

Subscribes
    detections   vision_msgs/Detection2DArray  (feature points, ideally with ids)
    camera_info  sensor_msgs/CameraInfo        (for the principal point)

Publishes
    ~/partition_ok  std_msgs/Bool              latch this in your servo loop
    /diagnostics    diagnostic_msgs/DiagnosticArray

This node does NOT control anything. Keep your controller; gate it on the Bool:

    if partition_ok:  v = my_partitioned_control(...)
    else:             v = my_plain_control(...)

There is no default threshold. `calibration` is required and must come from a
KNOWN-GOOD target, because a threshold calibrated on a degenerate one never
fires and nothing looks wrong.
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray

from blindspot import FeatureGuard, intrinsics_from_fov
from blindspot.ros_bridge import (detection_center, detection_label,
                                  features_to_normalised,
                                  intrinsics_from_camera_info,
                                  order_features, ordering_is_suspect)


class GuardNode(Node):

    def __init__(self):
        super().__init__("blindspot_guard")
        self.declare_parameter("calibration", "")
        self.declare_parameter("fovy_deg", 0.0)
        self.declare_parameter("expected_ids", [])

        path = self.get_parameter("calibration").value
        if not path:
            raise RuntimeError(
                "parameter 'calibration' is required; this node ships no "
                "default threshold. Calibrate on a KNOWN-GOOD target:\n"
                "  ros2 run blindspot calibrate --geometry target.json "
                "--goal-pose pose.json -o my_target.json")
        self.guard = FeatureGuard.load(path)
        self.get_logger().info(
            "loaded %s, thresholds by visible count: %s"
            % (path, {n: "%.3e" % v for n, v in
                      sorted(self.guard.calibration.thresholds.items())}))

        self.fovy = float(self.get_parameter("fovy_deg").value)
        self.expected = [str(i) for i in
                         self.get_parameter("expected_ids").value]
        self.K = None
        self.size = None
        self.warned_uncal = False

        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(CameraInfo, "camera_info", self.on_info,
                                 latched)
        self.create_subscription(Detection2DArray, "detections",
                                 self.on_detections, 10)
        self.pub_ok = self.create_publisher(Bool, "~/partition_ok", 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics",
                                              10)

    def on_info(self, msg):
        self.K = msg.k
        self.size = (msg.width, msg.height)

    def intrinsics(self):
        if self.size is None:
            return None
        fx, fy, cx, cy, src = intrinsics_from_camera_info(self.K, *self.size)
        if fx is None:
            if self.fovy <= 0:
                return None
            fx, fy, cx, cy = intrinsics_from_fov(self.size[0], self.size[1],
                                                 self.fovy)
            src = "fovy_deg parameter"
            if not self.warned_uncal:
                self.get_logger().warn(
                    "camera_info carries no intrinsics; using fovy_deg=%.1f "
                    "with principal point (W-1)/2" % self.fovy)
                self.warned_uncal = True
        return fx, fy, cx, cy, src

    def on_detections(self, msg):
        intr = self.intrinsics()
        if intr is None:
            self.publish_diag(DiagnosticStatus.ERROR,
                              "waiting for camera_info (or set fovy_deg)", [])
            return
        fx, fy, cx, cy, src = intr

        uv, ids = [], []
        for det in msg.detections:
            did = detection_label(det)
            if self.expected and str(did) not in self.expected:
                continue
            uv.append(detection_center(det))
            ids.append(did)
        if not uv:
            self.publish(False, DiagnosticStatus.WARN, "no features detected",
                         [("n_features", "0")])
            return

        uv = np.asarray(uv, dtype=float)
        have_ids = all(i is not None for i in ids)
        s, _, how = features_to_normalised(uv, fx, fy, cx, cy,
                                           ids if have_ids else None)
        ordered_uv, _, _ = order_features(uv, ids if have_ids else None)
        suspect = ordering_is_suspect(ordered_uv)

        r = self.guard.evaluate(s)
        level = DiagnosticStatus.OK if r.decision else DiagnosticStatus.WARN
        text = ("partition ok" if r.decision else
                "partition DROPPED - use your plain control law")
        if suspect:
            level = DiagnosticStatus.ERROR
            text = ("feature ordering crosses itself, so the area is not "
                    "measuring the geometry; the decision is unreliable")
        self.publish(r.decision, level, text, [
            ("signal", "%.6e" % r.signal),
            ("threshold", "%.6e" % r.threshold),
            ("margin", "%.3f" % r.margin),
            ("n_features", str(r.n_features)),
            ("decision", "partition_ok" if r.decision else "drop_partition"),
            ("ordering", how),
            ("intrinsics", src),
        ])

    def publish(self, ok, level, text, kvs):
        self.pub_ok.publish(Bool(data=bool(ok)))
        self.publish_diag(level, text, kvs)

    def publish_diag(self, level, text, kvs):
        st = DiagnosticStatus(level=level, name="blindspot: partition guard",
                              hardware_id="blindspot",
                              message=text,
                              values=[KeyValue(key=k, value=v)
                                      for k, v in kvs])
        arr = DiagnosticArray(status=[st])
        arr.header.stamp = self.get_clock().now().to_msg()
        self.pub_diag.publish(arr)


def main(argv=None):
    rclpy.init(args=argv)
    try:
        node = GuardNode()
    except RuntimeError as e:
        print("blindspot_guard: %s" % e)
        _shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy >= Lyrical shuts the context down in its own SIGINT
        # handler and spin() then raises ExternalShutdownException
        # rather than KeyboardInterrupt. Catching only the latter
        # exits 1 on a clean Ctrl-C - measured on Lyrical, where
        # Jazzy had shown a clean exit.
        pass
    finally:
        node.destroy_node()
        _shutdown()
    return 0


def _shutdown():
    """Shut down only if nobody has already.

    ros2 launch delivers SIGINT and rclpy may already have torn the context
    down by the time `finally` runs; calling shutdown again raises RCLError
    and the node exits 1 on a clean Ctrl-C.
    """
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    raise SystemExit(main())
