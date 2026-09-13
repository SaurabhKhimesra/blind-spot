"""A rendered-camera feature source that is drop-in for project().

The ONLY thing this changes versus the numpy study is where feature positions
come from. Same free-flying camera, same cTo, same control loop, same
scenarios. No arm, no IK, no QP solver - those are a separate question
(joint limits, dynamics, contact) and would change four variables at once.

Scene: six ArUco markers laid out as the SAME hexagon the numpy work uses
(ring(r=0.05, n=6)), on a white plane. One feature per marker: its centre,
taken as the mean of the four detected corners. That maps one-to-one onto the
existing 6-point ring, so every scenario ports directly - occlusion is a box
drawn over N markers, collapse is the same target flattening.

Camera conventions
------------------
The IBVS code uses the computer-vision frame: +z forward, +x right, +y down.
MuJoCo cameras use the OpenGL frame: -z forward, +x right, +y up. The two
differ by diag(1, -1, -1), applied in mj_camera_from_cTo. That conversion is
NOT reasoned about - validate.py measures detected features against project()
and the agreement is reported as a number before anything else is believed.
"""

import os
import numpy as np
import cv2
import mujoco

MARKER_M = 0.03           # marker edge length, metres (hexagon side is 0.05)
RING_R = 0.05
WIDTH, HEIGHT = 1920, 1440
FOVY_DEG = 45.0

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
CV_FROM_GL = np.diag([1.0, -1.0, -1.0])     # y and z flip


def intrinsics(width=WIDTH, height=HEIGHT, fovy_deg=FOVY_DEG):
    """Pinhole intrinsics matching MuJoCo's vertical-FOV camera.

    The principal point is (W-1)/2, not W/2. OpenGL maps NDC [-1,1] onto a
    CONTINUOUS pixel range [0, W], so the optical axis lands at continuous
    coordinate W/2; OpenCV indexes pixel CENTRES, which sit half a pixel
    lower. Using W/2 shows up as a systematic (-0.49, -0.50) px bias in the
    detected features - measured, and exactly the half pixel this predicts.
    """
    fy = (height / 2.0) / np.tan(np.deg2rad(fovy_deg) / 2.0)
    return fy, fy, (width - 1) / 2.0, (height - 1) / 2.0   # fx, fy, cx, cy


def pixels_to_normalised(uv, width=WIDTH, height=HEIGHT, fovy_deg=FOVY_DEG):
    """Pixel coords -> the normalised image coords project() returns."""
    fx, fy, cx, cy = intrinsics(width, height, fovy_deg)
    return np.stack([(uv[:, 0] - cx) / fx, (uv[:, 1] - cy) / fy], axis=1)


def ring(r=RING_R, n=6):
    a = np.linspace(0, 2 * np.pi, n + 1)[:-1]
    return np.stack([r * np.cos(a), r * np.sin(a), np.zeros(n)], axis=1)


def write_marker_pngs(outdir, n=6, px=400, quiet=80):
    """ArUco markers with a white quiet zone, which the detector requires.

    The texture is written MIRRORED. The IBVS convention views this plane from
    object-frame -z, and a marker seen from behind is mirrored; an ArUco
    dictionary is not mirror-invariant, so an unmirrored texture renders a
    marker that is physically correct but undetectable (measured: 0 of 6
    detected as rendered, 6 of 6 after flipping the image). Mirroring the
    texture instead of flipping the captured frame keeps the detector looking
    at a genuine, correctly-oriented marker - flipping the measurement would
    hide a real geometry error rather than fix one.
    """
    os.makedirs(outdir, exist_ok=True)
    paths = []
    for i in range(n):
        img = cv2.flip(cv2.aruco.generateImageMarker(ARUCO_DICT, i, px), 1)
        pad = np.full((px + 2 * quiet, px + 2 * quiet), 255, np.uint8)
        pad[quiet:quiet + px, quiet:quiet + px] = img
        p = os.path.join(outdir, "marker_%d.png" % i)
        cv2.imwrite(p, pad)
        paths.append(p)
    return paths


def scene_xml(marker_paths, P_o, marker_m=MARKER_M, occluders=()):
    """MuJoCo scene: markers at P_o on a white plane, plus optional occluders.

    occluders : iterable of (x, y, half_w, half_h) boxes in the object plane,
                drawn just above the markers. This is how occlusion is applied
                - a real box in the scene, not a mask applied to the answer.
    """
    half = marker_m / 2.0
    # The IBVS convention puts the camera at object-frame -z looking toward
    # +z, so everything must face -z and the backdrop must sit BEHIND the
    # markers in +z. Marker boxes are centred at +TH so their front face lands
    # exactly on z=0: the detected centre then corresponds to P_o with no
    # depth offset, which the validation gate would otherwise pick up.
    TH = 0.0004
    tex = "\n".join(
        '    <texture name="t%d" type="2d" file="%s"/>\n'
        '    <material name="m%d" texture="t%d" texuniform="false"'
        ' specular="0" shininess="0" reflectance="0"/>' % (i, p, i, i)
        for i, p in enumerate(marker_paths))
    bodies = "\n".join(
        '    <geom name="mk%d" type="box" pos="%.6f %.6f %.6f"'
        ' size="%.6f %.6f %.6f" material="m%d"/>'
        % (i, P_o[i, 0], P_o[i, 1], P_o[i, 2] + TH, half, half, TH, i)
        for i in range(len(marker_paths)))
    occ = "\n".join(
        '    <geom name="occ%d" type="box" pos="%.6f %.6f -0.010"'
        ' size="%.6f %.6f 0.004" rgba="0.15 0.15 0.17 1"/>'
        % (j, o[0], o[1], o[2], o[3]) for j, o in enumerate(occluders))
    return """
<mujoco model="ibvs_aruco">
  <compiler texturedir="."/>
  <!-- extent is pinned: MuJoCo derives znear/zfar from an AUTO-computed
       extent, so changing marker size or collapsing the target silently moved
       the near plane and clipped the whole scene (image mean 254 -> 13).
       Rendering must not depend on the geometry under test. -->
  <statistic extent="1.0" center="0 0 0"/>
  <visual>
    <map znear="0.005" zfar="50"/>
    <global offwidth="%d" offheight="%d"/>
    <quality shadowsize="0"/>
    <headlight ambient="0.6 0.6 0.6" diffuse="0.5 0.5 0.5"
               specular="0 0 0"/>
  </visual>
  <asset>
%s
    <material name="ground" rgba="1 1 1 1" specular="0" shininess="0"/>
  </asset>
  <worldbody>
    <geom name="floor" type="box" pos="0 0 0.0028" size="0.6 0.6 0.002"
          material="ground"/>
%s
%s
    <body name="cambody" mocap="true" pos="0 0 1">
      <camera name="cam" fovy="%.4f" pos="0 0 0"
              xyaxes="1 0 0 0 1 0"/>
    </body>
  </worldbody>
</mujoco>
""" % (WIDTH, HEIGHT, tex, bodies, occ, FOVY_DEG)


def mj_camera_from_cTo(cTo):
    """Camera mocap pos/quat (world frame) from the IBVS cTo transform.

    cTo maps object-frame points into the CV camera frame. The object frame is
    the world frame here, so the camera's world pose is the inverse, then
    converted from the CV frame to MuJoCo's OpenGL camera frame.
    """
    R_co, t_co = cTo[:3, :3], cTo[:3, 3]
    R_oc = R_co.T                       # world <- cv-camera
    pos = -R_oc @ t_co                  # camera centre in world
    R_gl = R_oc @ CV_FROM_GL            # world <- gl-camera
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.ascontiguousarray(R_gl.reshape(-1)))
    return pos, quat


class RenderedFeatureSource:
    """Renders the scene at a given cTo and detects marker-centre features."""

    def __init__(self, P_o, occluders=(), marker_m=MARKER_M, texdir=None):
        texdir = texdir or os.path.join(
            os.environ.get("BLINDSPOT_TEX", "."), "_aruco_tex")
        paths = write_marker_pngs(texdir, n=P_o.shape[0])
        rel = [os.path.relpath(p, texdir) for p in paths]
        xml = scene_xml(rel, P_o, marker_m, occluders)
        self.model = mujoco.MjModel.from_xml_string(
            xml.replace('texturedir="."', 'texturedir="%s"' % texdir))
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, HEIGHT, WIDTH)
        params = cv2.aruco.DetectorParameters()
        # SUBPIX measured at 0.084 px mean with zero bias. APRILTAG refinement
        # uses a corner convention offset by ~half a pixel and was cancelling
        # the principal-point bug above - two errors hiding each other.
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(ARUCO_DICT, params)
        self.n = P_o.shape[0]

    def close(self):
        """Release the renderer's GL context.

        Renderers must be closed when finished. Live MjRenderer instances
        accumulate GL resources and rendering degrades SILENTLY once enough
        are open - frames come back visually plausible but undetectable. A
        sweep that built ~30 sources in one process reported 0/6 detections
        for every scene after the first, which looked exactly like a physical
        limit of the target geometry and was not one.
        """
        try:
            self.renderer.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def render(self, cTo):
        pos, quat = mj_camera_from_cTo(cTo)
        self.data.mocap_pos[0] = pos
        self.data.mocap_quat[0] = quat
        mujoco.mj_forward(self.model, self.data)
        self.renderer.update_scene(self.data, camera="cam")
        return self.renderer.render()

    def detect(self, rgb):
        """Return (features_normalised (n,2) with NaN where missing, vis mask)."""
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        uv = np.full((self.n, 2), np.nan)
        vis = np.zeros(self.n, dtype=bool)
        if ids is not None:
            for c, i in zip(corners, ids.ravel()):
                if 0 <= i < self.n:
                    uv[i] = c[0].mean(axis=0)       # marker centre
                    vis[i] = True
        s = np.full((self.n, 2), np.nan)
        good = ~np.isnan(uv[:, 0])
        if good.any():
            s[good] = pixels_to_normalised(uv[good])
        return s, vis

    def features(self, cTo):
        return self.detect(self.render(cTo))
