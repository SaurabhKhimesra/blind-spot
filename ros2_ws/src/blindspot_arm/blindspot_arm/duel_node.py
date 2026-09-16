"""Two UR5e cells doing the same insertion, one with the guard, one without.

LEFT  cell: the 2001 partition stays on, always. This is what you get if your
            safety logic counts features, because the count never drops.
RIGHT cell: the feature guard decides, per step.

Both cells see the same panel tilt at the same sim time, share one physics
clock, and run identical code apart from that one decision.

The camera twist is mapped to joint velocities through the arm's own geometric
Jacobian (blindspot_arm.kin, FK verified against TF to 1e-6 m), so joint limits
and the real manipulator geometry are in the loop.
"""

import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from blindspot import FeatureGuard
from blindspot.calibrate import calibrate
from blindspot_arm.kin import Chain

R_TARGET = 0.05          # fiducial ring radius on the panel, metres
Z_DESIRED = 0.26         # standoff the servo drives to
LAM = 0.6
TAU = 1e-3
DT = 0.1
XACRO = ("/home/saurabh/blind-spot/ros2_ws/install/blindspot_arm/share/"
         "blindspot_arm/urdf/ur5e_camera.urdf.xacro")

# Intrinsics come from the sensor definition in the URDF, not from
# camera_info. The gz topic /left_wrist_camera_info exists but nothing arrives
# on the ROS side, and waiting on it silently stalled the whole servo loop:
# step() returned early, so the HUD read 0/6 features and neither arm moved.
# Principal point is (W-1)/2, the convention used throughout this repo.
IMG_W, IMG_H, HFOV = 640, 480, 1.0472
_FX = (IMG_W / 2.0) / np.tan(HFOV / 2.0)
K_DEFAULT = (_FX, _FX, (IMG_W - 1) / 2.0, (IMG_H - 1) / 2.0)

# act timeline, in seconds of sim time.
# The arm MUST be driven back to home and allowed to converge before the tilt,
# otherwise a run inherits wherever the previous one stopped and the tilt
# begins from an unconverged pose - which looked like the guard failing when
# it was really the approach never having happened.
T_HOME, T_TILT, T_RAMP, T_END = 8.0, 22.0, 9.0, 46.0
TILT_MAX = 1.15          # radians. Tuned: enough to drive the polygon
                         # area under the guard threshold, not so far that
                         # a dot leaves the frame. The story needs all six
                         # visible while the geometry degenerates.


def hexagon(r):
    a = np.linspace(0, 2 * np.pi, 7)[:-1]
    return np.stack([r * np.cos(a), r * np.sin(a), np.zeros(6)], axis=1)


def order_by_angle(pts):
    c = pts.mean(axis=0)
    return pts[np.argsort(np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0]))]


def best_cyclic_match(s, s_star):
    """Rotate s_star to the cyclic shift that actually pairs with s.

    Ordering six identical dots by angle fixes their ORDER but not where the
    order starts, so a rotated hexagon can pair every dot with the wrong
    target. The arm then chases a bogus goal: measured, one cell converged to
    |e| 0.090 while its twin diverged to 0.530 running identical code. Picking
    the shift with the smallest residual removes the ambiguity.
    """
    best, best_err = s_star, np.inf
    for k in range(len(s_star)):
        cand = np.roll(s_star, k, axis=0)
        err = np.linalg.norm(s - cand)
        if err < best_err:
            best, best_err = cand, err
    return best


def detect_dots(rgb):
    g = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    _, th = cv2.threshold(g, 90, 255, cv2.THRESH_BINARY_INV)
    n, _, stats, cent = cv2.connectedComponentsWithStats(th, 8)
    cand = [(int(s[4]), c) for s, c in zip(stats[1:], cent[1:])
            if 25 < s[4] < 4000]
    if not cand:
        return np.zeros((0, 2))
    # The panel carries exactly six dots. Rendering artefacts and the post
    # edge can add spurious components, so keep the six largest: a 7th blob
    # otherwise breaks correspondence against the six desired features.
    # The six dots are identical circles, so pick the six most SIMILAR
    # blobs, not the six largest. Taking the largest lets the robot's own
    # dark wrist - which is in frame at close standoff - masquerade as a
    # fiducial and displace a real dot. Offline the control law converges to
    # |e| 0.0008 on ground-truth features, so divergence in the sim was
    # perception, not control.
    if len(cand) < 6:
        return np.array([c for _, c in cand], dtype=float)
    cand.sort(key=lambda x: x[0])
    best, spread = None, np.inf
    for i in range(len(cand) - 5):
        win = cand[i:i + 6]
        sp = win[-1][0] / max(win[0][0], 1)
        if sp < spread:
            best, spread = win, sp
    return np.array([c for _, c in best], dtype=float)


def interaction(s, Z):
    L = np.zeros((2 * len(s), 6))
    for i, (x, y) in enumerate(s):
        L[2 * i] = [-1 / Z, 0, x / Z, x * y, -(1 + x * x), y]
        L[2 * i + 1] = [0, -1 / Z, y / Z, 1 + y * y, -x * y, -x]
    return L


def pinv_trunc(L, tau):
    U, S, Vt = np.linalg.svd(L, full_matrices=False)
    keep = S > tau * S[0]
    Si = np.where(keep, 1.0 / np.where(S > 0, S, 1.0), 0.0)
    return (Vt.T * Si) @ U.T


def poly_sigma(s):
    x, y = s[:, 0], s[:, 1]
    return float(np.sqrt(max(0.5 * abs(np.dot(x, np.roll(y, -1))
                                       - np.dot(y, np.roll(x, -1))), 1e-12)))


def ibvs_twist(s, s_star, use_partition):
    """Camera twist. Partitioned when the partition is safe, plain otherwise."""
    e = (s - s_star).reshape(-1)
    L = interaction(s, Z_DESIRED)
    if use_partition and len(s) >= 3:
        sig, sig_star = poly_sigma(s), poly_sigma(s_star)
        vz = 0.6 * np.log(sig_star / max(sig, 1e-9))
        al = np.arctan2(s[1, 1] - s[0, 1], s[1, 0] - s[0, 0])
        al_s = np.arctan2(s_star[1, 1] - s_star[0, 1],
                          s_star[1, 0] - s_star[0, 0])
        wz = 0.6 * ((al - al_s + np.pi) % (2 * np.pi) - np.pi)
        xy, z = [0, 1, 3, 4], [2, 5]
        v_xy = -pinv_trunc(L[:, xy], TAU) @ (LAM * e + L[:, z] @ [vz, wz])
        v = np.zeros(6)
        v[xy], v[2], v[5] = v_xy, vz, wz
        return v
    return -LAM * pinv_trunc(L, TAU) @ e


class Cell:
    def __init__(self, node, side, guard, guarded):
        self.side, self.guard, self.guarded = side, guard, guarded
        self.K = K_DEFAULT
        self.img = None
        self.q = None
        self.reading = None
        self.twist_norm = 0.0
        self.err = 0.0
        self.n_feat = 0
        desc = subprocess.run(
            ["xacro", XACRO, "ur_type:=ur5e", "prefix:=%s_" % side,
             "ns:=/%s" % side], capture_output=True, text=True).stdout
        self.chain = Chain(desc, "world", "%s_camera_optical_frame" % side)
        self.names = self.chain.joint_names
        self.pub = node.create_publisher(
            JointTrajectory, "/%s/arm_controller/joint_trajectory" % side, 10)
        self.tilt = node.create_publisher(Float64, "/panel_tilt_%s" % side, 10)
        node.create_subscription(CameraInfo, "/%s/camera/camera_info" % side,
                                 self.on_info, 10)
        node.create_subscription(Image, "/%s/camera/image" % side,
                                 self.on_img, 10)
        from sensor_msgs.msg import JointState
        node.create_subscription(JointState, "/%s/joint_states" % side,
                                 self.on_js, 10)

    def on_info(self, m):
        if m.k[0] > 0:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def on_img(self, m):
        self.img = np.frombuffer(m.data, np.uint8).reshape(
            m.height, m.width, 3).copy()

    def on_js(self, m):
        d = dict(zip(m.name, m.position))
        if all(n in d for n in self.names):
            self.q = np.array([d[n] for n in self.names])

    def go_home(self):
        """Drive to a known pose so every run starts identically."""
        msg = JointTrajectory()
        msg.joint_names = self.names
        pt = JointTrajectoryPoint()
        pt.positions = [0.0, -1.2, 1.4, -1.75, -1.57, 0.0]
        pt.time_from_start.sec = 3
        msg.points.append(pt)
        self.pub.publish(msg)

    def step(self):
        if self.img is None or self.K is None or self.q is None:
            return
        fx, fy, cx, cy = self.K
        uv = detect_dots(self.img)
        self.n_feat = len(uv)
        if len(uv) < 3:
            return
        uv = order_by_angle(uv)
        s = np.stack([(uv[:, 0] - cx) / fx, (uv[:, 1] - cy) / fy], axis=1)
        self.reading = self.guard.evaluate(s)

        # Servo only on the full set. With fewer than six the angular ordering
        # of what is seen does not correspond to the six desired features, and
        # servoing on a mismatched pairing would be meaningless. The guard
        # still reports, which is the point.
        if len(s) != 6:
            self.twist_norm = 0.0
            return
        s_star = best_cyclic_match(
            s, order_by_angle(hexagon(R_TARGET / Z_DESIRED)[:, :2]))
        # LEFT cell ignores the guard entirely: partition always on.
        use_partition = self.reading.decision if self.guarded else True

        v_cam = ibvs_twist(s, s_star, use_partition)
        self.err = float(np.linalg.norm((s - s_star).reshape(-1)))

        T = self.chain.fk(self.q)
        Rbc = T[:3, :3]
        tw = np.concatenate([Rbc @ v_cam[:3], Rbc @ v_cam[3:]])
        J = self.chain.jacobian(self.q)
        qd = np.linalg.pinv(J) @ tw
        qd = np.clip(qd, -1.5, 1.5)
        self.twist_norm = float(np.linalg.norm(v_cam))

        self.dbg = (self.n_feat, self.err, float(np.linalg.norm(qd)),
                    float(np.linalg.norm(self.q)))
        q_next = self.q + qd * DT
        msg = JointTrajectory()
        msg.joint_names = self.names
        pt = JointTrajectoryPoint()
        pt.positions = q_next.tolist()
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = int(DT * 1.6e9)
        msg.points.append(pt)
        self.pub.publish(msg)

    def set_tilt(self, a):
        self.tilt.publish(Float64(data=float(a)))


class Duel(Node):

    def __init__(self):
        super().__init__("blindspot_duel")
        self.get_logger().info("calibrating guard on the known-good panel...")
        goal = np.eye(4)
        goal[2, 3] = Z_DESIRED
        cal = calibrate(hexagon(R_TARGET), goal, n_poses=600, seed=0,
                        z_range=(0.18, 0.40), lateral=0.05)
        guard = FeatureGuard(cal)
        self.get_logger().info(
            "thresholds: %s" % {n: "%.3e" % v
                                for n, v in sorted(cal.thresholds.items())})
        self.left = Cell(self, "left", guard, guarded=False)
        self.right = Cell(self, "right", guard, guarded=True)
        self.hud = self.create_publisher(Image, "/duel/hud", 10)
        self.t0 = None
        self.create_timer(DT, self.tick)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        # the act: home, converge, then the panel swings toward edge-on,
        # identically on both sides
        a = 0.0 if t < T_TILT else TILT_MAX * min(1.0, (t - T_TILT) / T_RAMP)
        for c in (self.left, self.right):
            c.set_tilt(a)
            if t < T_HOME:
                c.go_home()
            else:
                c.step()
        if int(t * 10) % 10 == 0 and t < T_TILT + 2:
            for nm, c in (("L", self.left), ("R", self.right)):
                d = getattr(c, "dbg", None)
                if d:
                    self.get_logger().info(
                        "%s t=%5.1f n=%d |e|=%.3f |qd|=%.3f |q|=%.2f"
                        % (nm, t, d[0], d[1], d[2], d[3]))
        self.publish_hud(t, a)

    def publish_hud(self, t, a):
        panes = []
        for c, title, col in ((self.left, "GUARD OFF", (235, 70, 55)),
                              (self.right, "GUARD ON", (60, 200, 90))):
            img = (c.img.copy() if c.img is not None
                   else np.full((480, 640, 3), 30, np.uint8))
            ok = c.reading.decision if c.reading else True
            state = "PARTITION ON" if (not c.guarded or ok) else "PARTITION DROPPED"
            bar = col if (not c.guarded) else ((60, 200, 90) if ok else (235, 70, 55))
            img[:38] = (38, 38, 46)
            cv2.putText(img, title, (12, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        col, 2, cv2.LINE_AA)
            img[38:44] = bar
            img[-64:] = (38, 38, 46)
            cv2.putText(img, "features %d/6   |e| %.3f" % (c.n_feat, c.err),
                        (12, 480 - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.56,
                        (225, 225, 235), 1, cv2.LINE_AA)
            m = c.reading.margin if c.reading else 0.0
            cv2.putText(img, "%s   margin %.2fx   |v| %.2f"
                        % (state, m, c.twist_norm),
                        (12, 480 - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.56,
                        bar, 2, cv2.LINE_AA)
            panes.append(img)
        gap = np.full((480, 8, 3), 255, np.uint8)
        row = np.hstack([panes[0], gap, panes[1]])
        head = np.full((46, row.shape[1], 3), 255, np.uint8)
        phase = ("returning to home" if t < T_HOME else
                 "approach - both cells identical" if t < T_TILT else
                 "PANEL TILTING - geometry degenerating, markers still visible")
        cv2.putText(head, "t=%5.1fs   tilt %4.2f rad   %s" % (t, a, phase),
                    (12, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                    (25, 25, 30) if t < T_TILT else (200, 60, 20), 2,
                    cv2.LINE_AA)
        out = np.vstack([head, row])
        m = Image()
        m.header.stamp = self.get_clock().now().to_msg()
        m.height, m.width = out.shape[:2]
        m.encoding, m.step = "rgb8", out.shape[1] * 3
        m.data = out.tobytes()
        self.hud.publish(m)


def main(argv=None):
    rclpy.init(args=argv)
    n = Duel()
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
