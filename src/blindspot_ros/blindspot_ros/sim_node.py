"""Live IBVS simulation, closed through ROS around the guard node.

This is the whole point made visible: a servo loop whose control law is
chosen, every step, by the Bool the guard node publishes.

    partition_ok == True   -> partitioned law (2001), good on retreat
    partition_ok == False  -> plain truncated law, safe when features go bad

It cycles through three regimes so the guard visibly changes its mind:
healthy, occluded down to 2 markers, and a target collapsed toward a line.

Publishes
    detections      vision_msgs/Detection2DArray   shuffled, with ids
    camera_info     sensor_msgs/CameraInfo
    camera/image    sensor_msgs/Image              the view, with an overlay
    markers         visualization_msgs/MarkerArray target and camera, in 3D
    tf              world -> camera

Everything is drawn with numpy, so there is no cv_bridge or OpenCV dependency.
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from vision_msgs.msg import (BoundingBox2D, Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)
from visualization_msgs.msg import Marker, MarkerArray

from blindspot.reference import pinv_truncated

Z_COLS = [2, 5]           # vz, wz  - driven from image measurements
XY_COLS = [0, 1, 3, 4]    # vx, vy, wx, wy - from the reduced inverse


def polygon_area_sqrt(s):
    x, y = s[:, 0], s[:, 1]
    return np.sqrt(max(0.5 * abs(np.dot(x, np.roll(y, -1))
                                 - np.dot(y, np.roll(x, -1))), 1e-12))


def line_angle(s):
    return np.arctan2(s[1, 1] - s[0, 1], s[1, 0] - s[0, 0])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

W, H, FOVY = 640, 480, 45.0
DT = 0.1
PHASE = 60
TAU = 1e-3
LAM = 0.5

# a 5x7 bitmap font, enough for a heads-up overlay without pulling in OpenCV
GLYPH = {
    "0": ["01110", "10001", "10011", "10101", "11001", "10001", "01110"],
    "1": ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
    "3": ["11111", "00010", "00100", "00010", "00001", "10001", "01110"],
    "4": ["00010", "00110", "01010", "10010", "11111", "00010", "00010"],
    "5": ["11111", "10000", "11110", "00001", "00001", "10001", "01110"],
    "6": ["00110", "01000", "10000", "11110", "10001", "10001", "01110"],
    "7": ["11111", "00001", "00010", "00100", "01000", "01000", "01000"],
    "8": ["01110", "10001", "10001", "01110", "10001", "10001", "01110"],
    "9": ["01110", "10001", "10001", "01111", "00001", "00010", "01100"],
    ".": ["00000", "00000", "00000", "00000", "00000", "01100", "01100"],
    "x": ["00000", "00000", "10001", "01010", "00100", "01010", "10001"],
    "/": ["00001", "00010", "00010", "00100", "01000", "01000", "10000"],
    "-": ["00000", "00000", "00000", "11111", "00000", "00000", "00000"],
    " ": ["00000"] * 7,
}
for _c, _r in zip("ABCDEFGHIJKLMNOPQRSTUVWXYZ",
                  ["01110 10001 10001 11111 10001 10001 10001",
                   "11110 10001 10001 11110 10001 10001 11110",
                   "01110 10001 10000 10000 10000 10001 01110",
                   "11110 10001 10001 10001 10001 10001 11110",
                   "11111 10000 10000 11110 10000 10000 11111",
                   "11111 10000 10000 11110 10000 10000 10000",
                   "01110 10001 10000 10111 10001 10001 01111",
                   "10001 10001 10001 11111 10001 10001 10001",
                   "11111 00100 00100 00100 00100 00100 11111",
                   "00111 00010 00010 00010 00010 10010 01100",
                   "10001 10010 10100 11000 10100 10010 10001",
                   "10000 10000 10000 10000 10000 10000 11111",
                   "10001 11011 10101 10101 10001 10001 10001",
                   "10001 11001 10101 10011 10001 10001 10001",
                   "01110 10001 10001 10001 10001 10001 01110",
                   "11110 10001 10001 11110 10000 10000 10000",
                   "01110 10001 10001 10001 10101 10010 01101",
                   "11110 10001 10001 11110 10100 10010 10001",
                   "01111 10000 10000 01110 00001 00001 11110",
                   "11111 00100 00100 00100 00100 00100 00100",
                   "10001 10001 10001 10001 10001 10001 01110",
                   "10001 10001 10001 10001 10001 01010 00100",
                   "10001 10001 10001 10101 10101 11011 10001",
                   "10001 10001 01010 00100 01010 10001 10001",
                   "10001 10001 01010 00100 00100 00100 00100",
                   "11111 00001 00010 00100 01000 10000 11111"]):
    GLYPH[_c] = _r.split()


def draw_text(img, x, y, text, colour, scale=2):
    for ch in text.upper():
        g = GLYPH.get(ch, GLYPH[" "])
        for r, row in enumerate(g):
            for c, bit in enumerate(row):
                if bit == "1":
                    y0, x0 = y + r * scale, x + c * scale
                    img[y0:y0 + scale, x0:x0 + scale] = colour
        x += 6 * scale
    return x


def disc(img, cx, cy, rad, colour):
    h, w = img.shape[:2]
    y0, y1 = max(0, int(cy - rad)), min(h, int(cy + rad) + 1)
    x0, x1 = max(0, int(cx - rad)), min(w, int(cx + rad) + 1)
    if y1 <= y0 or x1 <= x0:
        return
    yy, xx = np.mgrid[y0:y1, x0:x1]
    img[y0:y1, x0:x1][(yy - cy) ** 2 + (xx - cx) ** 2 <= rad * rad] = colour


def line(img, p, q, colour, width=1):
    n = int(max(abs(q[0] - p[0]), abs(q[1] - p[1]))) + 1
    for t in np.linspace(0, 1, n):
        disc(img, p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t,
             width, colour)


def ring(r=0.05, n=6):
    a = np.linspace(0, 2 * np.pi, n + 1)[:-1]
    return np.stack([r * np.cos(a), r * np.sin(a), np.zeros(n)], axis=1)


def skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def se3_exp(xi):
    v, w = np.asarray(xi[:3], float), np.asarray(xi[3:], float)
    th = np.linalg.norm(w)
    T = np.eye(4)
    if th < 1e-12:
        T[:3, 3] = v
        return T
    Wm = skew(w / th)
    T[:3, :3] = np.eye(3) + np.sin(th) * Wm + (1 - np.cos(th)) * (Wm @ Wm)
    V = (np.eye(3) + ((1 - np.cos(th)) / th) * Wm
         + ((th - np.sin(th)) / th) * (Wm @ Wm))
    T[:3, 3] = V @ v
    return T


def interaction_matrix(s, Z):
    L = np.zeros((2 * len(s), 6))
    for i, ((x, y), Zi) in enumerate(zip(s, Z)):
        L[2 * i] = [-1 / Zi, 0, x / Zi, x * y, -(1 + x * x), y]
        L[2 * i + 1] = [0, -1 / Zi, y / Zi, 1 + y * y, -x * y, -x]
    return L


class Sim(Node):

    def __init__(self):
        super().__init__("blindspot_sim")
        fy = (H / 2.0) / np.tan(np.deg2rad(FOVY) / 2.0)
        self.K = (fy, fy, (W - 1) / 2.0, (H - 1) / 2.0)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.info_pub = self.create_publisher(CameraInfo, "camera_info",
                                              latched)
        self.det_pub = self.create_publisher(Detection2DArray, "detections",
                                             10)
        self.img_pub = self.create_publisher(Image, "camera/image", 10)
        self.mk_pub = self.create_publisher(MarkerArray, "markers", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Bool, "/blindspot_guard/partition_ok",
                                 self.on_guard, 10)
        self.partition_ok = True
        self.k = 0
        self.phase = -1
        self.reset()
        self.publish_info()
        self.create_timer(DT, self.tick)

    def on_guard(self, m):
        self.partition_ok = m.data

    def reset(self):
        """Fresh offset pose, so every phase shows a real approach.

        Without this the servo converges in a couple of seconds and then sits
        perfectly still, which looks identical to nothing happening.
        """
        self.cTo = np.eye(4)
        self.cTo[:3, :3] = se3_exp(np.array([0, 0, 0, 0.25, 0.15, 0.4]))[:3, :3]
        self.cTo[:3, 3] = [0.06, -0.05, 0.88]

    def target(self):
        phase = (self.k // PHASE) % 3
        pts = ring()
        if phase == 2:
            pts[:, 1] *= 0.02
        keep = [0, 3] if phase == 1 else list(range(6))
        return pts, keep, phase

    def publish_info(self):
        m = CameraInfo()
        m.width, m.height = W, H
        fx, fy, cx, cy = self.K
        m.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        m.header.frame_id = "camera"
        self.info_pub.publish(m)

    def tick(self):
        pts, keep, phase = self.target()
        if phase != self.phase:            # new regime, new approach to watch
            self.phase = phase
            self.reset()
            P0 = np.hstack([pts, np.ones((len(pts), 1))])
            P_c0 = (self.cTo @ P0.T).T[:, :3]
            if np.any(P_c0[:, 2] < 1e-3):
                self.reset()
        P = np.hstack([pts, np.ones((len(pts), 1))])
        P_c = (self.cTo @ P.T).T[:, :3]
        if np.any(P_c[:, 2] < 1e-3):
            self.reset()
            return
        s = np.stack([P_c[:, 0] / P_c[:, 2], P_c[:, 1] / P_c[:, 2]], axis=1)

        goal = np.eye(4)
        goal[2, 3] = 0.6
        Pg = (goal @ P.T).T[:, :3]
        s_star = np.stack([Pg[:, 0] / Pg[:, 2], Pg[:, 1] / Pg[:, 2]], axis=1)

        e = (s[keep] - s_star[keep]).reshape(-1)
        L = interaction_matrix(s[keep], P_c[keep, 2])

        if self.partition_ok and len(keep) >= 3:
            # the 2001 partition: vz from the polygon area, wz from the line
            # angle between two designated features, the other four DOF from
            # the reduced interaction matrix with the coupling term ADDED.
            sig = polygon_area_sqrt(s[keep])
            sig_star = polygon_area_sqrt(s_star[keep])
            vz = 0.6 * np.log(sig_star / max(sig, 1e-9))
            wz = 0.6 * wrap(line_angle(s[keep]) - line_angle(s_star[keep]))
            L_xy, L_z = L[:, XY_COLS], L[:, Z_COLS]
            L_xy_inv, _ = pinv_truncated(L_xy, TAU)
            v_xy = -L_xy_inv @ (LAM * e + L_z @ np.array([vz, wz]))
            v = np.zeros(6)
            v[XY_COLS] = v_xy
            v[2], v[5] = vz, wz
        else:
            # the guard dropped the partition: plain truncated law on full L
            Linv, _ = pinv_truncated(L, TAU)
            v = -LAM * Linv @ e
        v = np.clip(v, -3.0, 3.0)
        self.cTo = se3_exp(-v * DT) @ self.cTo

        self.publish_detections(s, keep)
        self.publish_image(s, keep, phase, float(np.linalg.norm(v)))
        self.publish_markers(pts, keep)
        self.publish_tf()
        self.k += 1

    def publish_detections(self, s, keep):
        fx, fy, cx, cy = self.K
        uv = s * np.array([fx, fy]) + np.array([cx, cy])
        msg = Detection2DArray()
        msg.header.frame_id = "camera"
        msg.header.stamp = self.get_clock().now().to_msg()
        order = list(keep)
        np.random.default_rng(self.k).shuffle(order)
        for i in order:
            d = Detection2D()
            d.id = str(i)
            b = BoundingBox2D()
            b.center.position.x = float(uv[i, 0])
            b.center.position.y = float(uv[i, 1])
            b.size_x = b.size_y = 30.0
            d.bbox = b
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = str(i)
            h.hypothesis.score = 1.0
            d.results.append(h)
            msg.detections.append(d)
        self.det_pub.publish(msg)

    def publish_image(self, s, keep, phase, speed):
        fx, fy, cx, cy = self.K
        uv = s * np.array([fx, fy]) + np.array([cx, cy])
        img = np.full((H, W, 3), 26, np.uint8)
        for gx in range(0, W, 40):
            img[:, gx] = 38
        for gy in range(0, H, 40):
            img[gy, :] = 38

        ok = self.partition_ok
        col = (60, 200, 90) if ok else (235, 90, 60)
        pts = [uv[i] for i in keep]
        if len(pts) >= 3:
            for a, b in zip(pts, pts[1:] + pts[:1]):
                line(img, a, b, (90, 90, 110), 1)
        for i in keep:
            disc(img, uv[i, 0], uv[i, 1], 7, col)
            disc(img, uv[i, 0], uv[i, 1], 3, (255, 255, 255))

        img[0:34] = (45, 45, 55)
        draw_text(img, 8, 10, ["HEALTHY 6 MARKERS", "OCCLUDED 2 MARKERS",
                               "COLLAPSED TARGET"][phase], (230, 230, 240), 2)
        img[H - 40:H] = (45, 45, 55)
        draw_text(img, 8, H - 32,
                  "PARTITION %s" % ("OK" if ok else "DROPPED"), col, 2)
        draw_text(img, 300, H - 32, "V %.2f" % speed, (200, 200, 210), 2)
        img[34:40] = col

        m = Image()
        m.header.frame_id = "camera"
        m.header.stamp = self.get_clock().now().to_msg()
        m.height, m.width = H, W
        m.encoding = "rgb8"
        m.step = W * 3
        m.data = img.tobytes()
        self.img_pub.publish(m)

    def publish_markers(self, pts, keep):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, p in enumerate(pts):
            mk = Marker()
            mk.header.frame_id = "world"
            mk.header.stamp = now
            mk.ns, mk.id, mk.type, mk.action = "target", i, Marker.SPHERE, 0
            mk.pose.position.x, mk.pose.position.y = float(p[0]), float(p[1])
            mk.pose.position.z = float(p[2])
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = mk.scale.z = 0.012
            seen = i in keep
            mk.color.r = 0.25 if seen else 0.55
            mk.color.g = 0.85 if seen else 0.55
            mk.color.b = 0.35 if seen else 0.55
            mk.color.a = 1.0 if seen else 0.25
            arr.markers.append(mk)
        self.mk_pub.publish(arr)

    def publish_tf(self):
        R = self.cTo[:3, :3].T
        t = -R @ self.cTo[:3, 3]
        q = np.empty(4)
        tr = np.trace(R)
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            q[3] = 0.25 * S
            q[0] = (R[2, 1] - R[1, 2]) / S
            q[1] = (R[0, 2] - R[2, 0]) / S
            q[2] = (R[1, 0] - R[0, 1]) / S
        else:
            q = np.array([0.0, 0.0, 0.0, 1.0])
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "world"
        tf.child_frame_id = "camera"
        tf.transform.translation.x = float(t[0])
        tf.transform.translation.y = float(t[1])
        tf.transform.translation.z = float(t[2])
        tf.transform.rotation.x, tf.transform.rotation.y = float(q[0]), float(q[1])
        tf.transform.rotation.z, tf.transform.rotation.w = float(q[2]), float(q[3])
        self.tf.sendTransform(tf)


def main(argv=None):
    rclpy.init(args=argv)
    n = Sim()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy >= Lyrical shuts the context down in its own SIGINT
        # handler and spin() then raises ExternalShutdownException
        # rather than KeyboardInterrupt. Catching only the latter
        # exits 1 on a clean Ctrl-C - measured on Lyrical, where
        # Jazzy had shown a clean exit.
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
