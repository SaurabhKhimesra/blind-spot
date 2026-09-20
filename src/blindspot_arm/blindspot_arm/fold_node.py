"""Two UR5e cells, one folding part: the case the 2001 partition cannot survive.

LEFT  cell: the 2001 partition stays on, always.
RIGHT cell: the feature guard decides, per step.

Mid-run both panels fold their two flaps away from the arm, so the six
markers collapse toward the hinge line IN 3D while all six stay in view. The
partition reads the shrinking polygon as "too far away" and drives the camera
at the part. Both cells run the same protective stop.

Measured first in numpy with this exact control law (blindspot.arm_law, and
the regression checks import the same module): at every
fold tested the unguarded law lunges to 2-8 cm and loses the target, plain
IBVS holds, sigma_6 never fires, and the area guard fires only when the fold
outpaces the partition's own depth loop (85 deg in <= 2 s). Slower folds are
regulated away by the lunge itself. This run uses 85 deg in 1 s.

What this act does NOT show, measured in the first Gazebo run and documented
in the README: held folded for several seconds, the guarded cell's fallback
creeps (a fully folded part leaves an orbit about the hinge line that the
image cannot see), and switching the partition back on during the unfold
produced a 6.05 command spike that lost the target. The act ends 2.5 s after
the fold completes for that reason (t_fold=20.0, a 1 s ramp, t_end=23.5), and the clip claims only what the act shows.

Everything runs on SIM time, so recording at a reduced real-time factor
changes nothing but wall-clock duration.
"""

import csv
import os
import subprocess

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ament_index_python.packages import get_package_share_directory

from blindspot import FeatureGuard
from blindspot.arm_law import (DT, K_DEFAULT, R_TARGET, Z_DESIRED,
                               best_cyclic_match, ibvs_twist, order_by_angle)
from blindspot.calibrate import calibrate
from blindspot_arm.detect import detect_dots
from blindspot_arm.kin import Chain

XACRO = os.path.join(get_package_share_directory("blindspot_arm"), "urdf",
                     "ur5e_camera.urdf.xacro")

PHI = np.deg2rad(11.0)   # marker layout rotation. At 0 deg two marker pairs
                         # share a column and merge in the image as the part
                         # folds; 11 deg keeps every column distinct.
PANEL = np.array([0.636, -0.250, 0.359])   # hinge centre, in each robot's frame

# act timeline, seconds of sim time (t_fold and t_end are ROS parameters)
T_HOME, FOLD_S = 8.0, 1.0
FOLD_MAX = np.deg2rad(85.0)
STOP_STANDOFF = 0.12     # protective stop, identical for both cells: no
                         # commanded motion may bring the camera closer than
                         # this to the hinge plane. Checked on the NEXT pose.


def hexagon(r):
    a = np.linspace(0, 2 * np.pi, 7)[:-1] + PHI
    return np.stack([r * np.cos(a), r * np.sin(a), np.zeros(6)], axis=1)


def fold_angle(t, t_fold):
    """0 until t_fold, then a 1 s ramp to FOLD_MAX, then held."""
    return FOLD_MAX * float(np.clip((t - t_fold) / FOLD_S, 0.0, 1.0))


class Cell:
    def __init__(self, node, side, guard, guarded):
        self.side, self.guard, self.guarded = side, guard, guarded
        self.K = K_DEFAULT
        self.img, self.q, self.reading = None, None, None
        self.v = np.zeros(6)
        self.err, self.n_feat, self.used_partition = 0.0, 0, True
        self.stopped_at = None
        self.lost = False
        desc = subprocess.run(
            ["xacro", XACRO, "ur_type:=ur5e", "prefix:=%s_" % side,
             "ns:=/%s" % side], capture_output=True, text=True).stdout
        self.chain = Chain(desc, "world", "%s_camera_optical_frame" % side)
        self.names = self.chain.joint_names
        self.pub = node.create_publisher(
            JointTrajectory, "/%s/arm_controller/joint_trajectory" % side, 10)
        self.fold = node.create_publisher(Float64, "/fold_%s" % side, 10)
        node.create_subscription(Image, "/%s/camera/image" % side,
                                 self.on_img, 10)
        node.create_subscription(JointState, "/%s/joint_states" % side,
                                 self.on_js, 10)

    def on_img(self, m):
        self.img = np.frombuffer(m.data, np.uint8).reshape(
            m.height, m.width, 3).copy()

    def on_js(self, m):
        d = dict(zip(m.name, m.position))
        if all(n in d for n in self.names):
            self.q = np.array([d[n] for n in self.names])

    def standoff(self, q):
        """Camera optical centre to the hinge plane, along the panel normal."""
        return float(self.chain.fk(q)[1, 3] - PANEL[1])

    def send(self, q, secs):
        msg = JointTrajectory()
        msg.joint_names = self.names
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in q]
        pt.time_from_start.sec = int(secs)
        pt.time_from_start.nanosec = int((secs - int(secs)) * 1e9)
        msg.points.append(pt)
        self.pub.publish(msg)

    def go_home(self):
        self.send([0.0, -1.2, 1.4, -1.75, -1.57, 0.0], 3.0)

    def step(self, t):
        self.v = np.zeros(6)
        if self.img is None or self.q is None or self.stopped_at is not None:
            return
        fx, fy, cx, cy = self.K
        uv = detect_dots(self.img)
        self.n_feat = len(uv)
        if len(uv) < 3:
            self.lost = True
            return
        uv = order_by_angle(uv)
        s = np.stack([(uv[:, 0] - cx) / fx, (uv[:, 1] - cy) / fy], axis=1)
        self.reading = self.guard.evaluate(s)
        if len(s) != 6:
            self.lost = True
            return
        self.lost = False
        s_star = best_cyclic_match(
            s, order_by_angle(hexagon(R_TARGET / Z_DESIRED)[:, :2]))
        self.used_partition = self.reading.decision if self.guarded else True
        v_cam = ibvs_twist(s, s_star, self.used_partition)
        self.err = float(np.linalg.norm((s - s_star).reshape(-1)))

        T = self.chain.fk(self.q)
        Rbc = T[:3, :3]
        tw = np.concatenate([Rbc @ v_cam[:3], Rbc @ v_cam[3:]])
        qd = np.clip(np.linalg.pinv(self.chain.jacobian(self.q)) @ tw,
                     -1.5, 1.5)
        q_next = self.q + qd * DT
        if self.standoff(q_next) < STOP_STANDOFF:
            # protective stop: hold where we are and latch, as a real cell
            # would until an operator resets it
            self.stopped_at = t
            self.send(self.q, 0.16)
            return
        self.v = v_cam
        self.send(q_next, DT * 1.6)


class Fold(Node):

    def __init__(self):
        super().__init__("blindspot_fold", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.declare_parameter("log", "/tmp/blindspot_fold_log.csv")
        self.declare_parameter("t_fold", 20.0)
        self.declare_parameter("t_end", 23.5)
        self.t_fold = float(self.get_parameter("t_fold").value)
        self.t_end = float(self.get_parameter("t_end").value)
        goal = np.eye(4)
        goal[2, 3] = Z_DESIRED
        cal = calibrate(hexagon(R_TARGET), goal, n_poses=600, seed=0,
                        z_range=(0.18, 0.40), lateral=0.05)
        guard = FeatureGuard(cal)
        self.get_logger().info("thresholds: %s" % {
            n: "%.3e" % v for n, v in sorted(cal.thresholds.items())})
        self.left = Cell(self, "left", guard, guarded=False)
        self.right = Cell(self, "right", guard, guarded=True)
        path = self.get_parameter("log").value
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.log_f = open(path, "w", newline="")
        self.log = csv.writer(self.log_f)
        self.log.writerow(["sim", "t", "fold", "side", "n_feat", "err", "signal",
                           "threshold", "margin", "partition", "v_norm",
                           "v_lin", "vz", "standoff", "stopped_at", "lost",
                           "cam_x", "cam_y", "cam_z"]
                          + ["q%d" % i for i in range(6)])
        self.t0 = None
        self.done = False
        self.create_timer(DT, self.tick)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:
            return                      # /clock not yet received
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        a = fold_angle(t, self.t_fold)
        for c in (self.left, self.right):
            c.fold.publish(Float64(data=float(a)))
            if t < T_HOME:
                c.go_home()
            else:
                c.step(t)
            if c.q is not None:
                r = c.reading
                self.log.writerow([
                    "%.3f" % now, "%.2f" % t, "%.4f" % a, c.side, c.n_feat, "%.5f" % c.err,
                    "%.5e" % (r.signal if r else 0), "%.5e" % (r.threshold if r else 0),
                    "%.4f" % (r.margin if r else 0), int(c.used_partition),
                    "%.4f" % np.linalg.norm(c.v), "%.4f" % np.linalg.norm(c.v[:3]),
                    "%.4f" % c.v[2], "%.4f" % c.standoff(c.q),
                    "" if c.stopped_at is None else "%.2f" % c.stopped_at,
                    int(c.lost)]
                    + ["%.4f" % x for x in c.chain.fk(c.q)[:3, 3]]
                    + ["%.5f" % x for x in c.q])
        self.log_f.flush()
        if int(t * 10) % 10 == 0 and t > T_HOME:
            self.get_logger().info(
                "t=%5.1f fold=%4.2f | L n=%d m=%.2f d=%.3f stop=%s | "
                "R n=%d m=%.2f d=%.3f part=%d stop=%s" % (
                    t, a, self.left.n_feat,
                    self.left.reading.margin if self.left.reading else 0,
                    self.left.standoff(self.left.q) if self.left.q is not None else 0,
                    self.left.stopped_at, self.right.n_feat,
                    self.right.reading.margin if self.right.reading else 0,
                    self.right.standoff(self.right.q) if self.right.q is not None else 0,
                    int(self.right.used_partition), self.right.stopped_at))
        if t > self.t_end and not self.done:
            self.done = True
            self.log_f.close()
            self.get_logger().info("act complete")
            raise SystemExit


def main(argv=None):
    rclpy.init(args=argv)
    n = Fold()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
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
