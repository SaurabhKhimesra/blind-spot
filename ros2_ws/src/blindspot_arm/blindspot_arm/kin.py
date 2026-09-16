"""Forward kinematics and geometric Jacobian for a serial chain, from URDF.

Pure numpy, no KDL. Written so it can be checked against TF rather than
trusted: verify_against_tf() compares this FK to what robot_state_publisher
reports, which is how a wrong axis or a missed fixed joint shows up.
"""

import numpy as np
from urdf_parser_py.urdf import URDF


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = (np.cos(r), np.sin(r), np.cos(p),
                              np.sin(p), np.cos(y), np.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def T_of(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(*rpy)
    T[:3, 3] = xyz
    return T


def axis_R(axis, q):
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)


class Chain:
    """Chain from `base` to `tip`, with the movable joints in order."""

    def __init__(self, urdf_xml, base, tip):
        self.robot = URDF.from_xml_string(urdf_xml)
        self.links = {l.name: l for l in self.robot.links}
        self.parent_joint = {j.child: j for j in self.robot.joints}
        self.steps = []          # (T_origin, axis or None) from base to tip
        self.joint_names = []
        node, chain = tip, []
        while node != base:
            j = self.parent_joint.get(node)
            if j is None:
                raise ValueError("no path from %s up to %s (stuck at %s)"
                                 % (tip, base, node))
            chain.append(j)
            node = j.parent
        for j in reversed(chain):
            org = j.origin
            T = T_of(org.xyz or [0, 0, 0], org.rpy or [0, 0, 0])
            if j.type in ("revolute", "continuous"):
                self.steps.append((T, np.asarray(j.axis, dtype=float)))
                self.joint_names.append(j.name)
            else:
                self.steps.append((T, None))

    def fk_all(self, q):
        """Returns (T_base_tip, [(z_i, p_i)]) with joint axes in base frame."""
        T = np.eye(4)
        axes = []
        k = 0
        for T_org, axis in self.steps:
            T = T @ T_org
            if axis is not None:
                z = T[:3, :3] @ (axis / np.linalg.norm(axis))
                axes.append((z, T[:3, 3].copy()))
                R = np.eye(4)
                R[:3, :3] = axis_R(axis, q[k])
                T = T @ R
                k += 1
        return T, axes

    def fk(self, q):
        return self.fk_all(q)[0]

    def jacobian(self, q):
        """Geometric Jacobian of the tip, expressed in the base frame."""
        T, axes = self.fk_all(q)
        p_tip = T[:3, 3]
        J = np.zeros((6, len(axes)))
        for i, (z, p) in enumerate(axes):
            J[:3, i] = np.cross(z, p_tip - p)
            J[3:, i] = z
        return J
