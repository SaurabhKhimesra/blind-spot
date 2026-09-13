"""Side-by-side video: a fixed controller failing, the guard-switched one not.

Same scene, same occlusion, same seed, same runs. Presentation only:

  * the story is t=0.2-2.4 s (22 control steps), so that window is rendered at
    ~5.6x slow motion and occupies most of the clip, with a short tail. The
    27-second hold on a frozen frame is gone.
  * the scene has a checkerboard backdrop and parallax structure, because a
    white void gives the eye nothing to track and a 5.5 m/s lurch then looks
    identical to standing still.
  * each panel carries a live log-scale |v| trace, drawn left to right on a
    shared axis so the 50x divergence is visible as two curves, not as a
    number changing between frames.
  * the occluder is a visible object and its entry is announced, so features
    dropping 6 -> 2 reads as an event rather than a glitch.

Slow motion is genuine: display frames are rendered at SE(3)-interpolated
poses between logged control steps, not duplicated.

    python3 mj_video.py            # writes docs/guard_vs_fixed.mp4
"""

import os
import numpy as np
import cv2
import imageio.v2 as imageio

from ibvs_core import make_pose, rot_z, rot_x, se3_exp
from partitioned import run_partitioned
from switched import run_switched, calibrate_area
from mj_scene import RenderedFeatureSource, ring, MARKER_M
from mj_cases import occluder_boxes
from mj_degrade import CorrelatedDegrader, se3_log

OUT = os.path.join("docs", "guard_vs_fixed.mp4")
DT, LAM, TAU = 0.1, 0.5, 1e-3
STEPS, OCC_FROM = 300, 4
OCC = (1, 2, 4, 5)
EXPOSURE = 0.04
FPS = 18
WIN = (2, 22)          # the steps where the whole story happens
SUB_SLOW, SUB_TAIL, SUB_SPIKE = 10, 2, 20
TAIL_TO = 30
HOLD = 16              # extra frames held on the worst-divergence step
PANEL = (500, 330)
TRACE_H = 110          # a third of the panel: this trace carries the story
CARD = 45              # title / end card length in frames (2.5 s)
REPO = "github.com/SaurabhKhimesra/blind-spot"

P_RING = ring()
CS = make_pose(t=np.array([0.0, 0.0, 0.6]))
CI = make_pose(R=rot_z(0.35) @ rot_x(0.15), t=np.array([0.08, -0.06, 0.9]))


class Rec:
    """Runs the controller, logging cTo and detections. No frames kept."""

    def __init__(self, P_o, occ_idx, expose):
        self.clear = RenderedFeatureSource(P_o, marker_m=MARKER_M,
                                           scenery=True)
        self.occl = RenderedFeatureSource(
            P_o, occluders=occluder_boxes(P_o, occ_idx), marker_m=MARKER_M,
            scenery=True)
        self.cTo, self.ndet = [], []
        self._cur = None
        outer = self

        class Sw:
            def render(self, c):
                return outer._cur.render(c)

            def detect(self, im):
                return outer._cur.detect(im)
        self.deg = CorrelatedDegrader(Sw(), exposure_frac=expose, P_o=P_o,
                                      max_sub=20)

    def __call__(self, k, cTo):
        self._cur = self.occl if k >= OCC_FROM else self.clear
        img = np.clip(self.deg._exposure_stack(cTo), 0, 255).astype(np.uint8)
        self.deg.prev_cTo = cTo.copy()
        s, det = self._cur.detect(img)
        self.cTo.append(cTo.copy())
        self.ndet.append(int(det.sum()))
        return s, det

    def close(self):
        self.clear.close()
        self.occl.close()


def interp(A, B, a):
    """Pose a fraction `a` of the way from A to B, along the SE(3) geodesic."""
    return se3_exp(se3_log(B @ np.linalg.inv(A)) * a) @ A


def trace_strip(width, vs, upto, colour, label, n_total, lo=-2.0, hi=1.0):
    """Log-scale |v| trace on a FIXED axis, revealed left to right.

    Both panels use the same lo/hi and the same n_total for the whole clip,
    and the axis is drawn up front with the line growing into it. If either
    axis rescaled mid-clip the two panels would stop being comparable - the
    same silent-comparison problem the figure avoids.
    """
    img = np.full((TRACE_H, width, 3), 252, np.uint8)
    L, R, TOP, BOT = 52, 10, 22, 16
    for dec, lab in [(1.0, "10"), (0.0, "1"), (-1.0, "0.1"), (-2.0, "0.01")]:
        y = int(TRACE_H - BOT - (dec - lo) / (hi - lo) * (TRACE_H - TOP - BOT))
        cv2.line(img, (L, y), (width - R, y), (214, 214, 214), 1)
        cv2.putText(img, lab, (6, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.44,
                    (110, 110, 110), 1, cv2.LINE_AA)
    cv2.line(img, (L, TOP - 6), (L, TRACE_H - BOT), (180, 180, 180), 1)
    for s in range(0, n_total + 1, 10):
        x = int(L + (width - L - R) * s / max(n_total - 1, 1))
        cv2.line(img, (x, TRACE_H - BOT), (x, TRACE_H - BOT + 4),
                 (180, 180, 180), 1)
        cv2.putText(img, "%.0fs" % (s * DT), (x - 10, TRACE_H - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (130, 130, 130), 1,
                    cv2.LINE_AA)
    cv2.putText(img, label, (L + 6, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.46,
                colour, 1, cv2.LINE_AA)

    n = max(n_total - 1, 1)
    pts = []
    for i, v in enumerate(vs[:max(upto, 1)]):
        x = int(L + (width - L - R) * i / n)
        d = np.clip(np.log10(max(v, 1e-3)), lo, hi)
        y = int(TRACE_H - BOT - (d - lo) / (hi - lo) * (TRACE_H - TOP - BOT))
        pts.append((x, y))
    if len(pts) > 1:
        cv2.polylines(img, [np.array(pts, np.int32)], False, colour, 3,
                      cv2.LINE_AA)
    if pts:
        cv2.circle(img, pts[-1], 5, colour, -1, cv2.LINE_AA)
    return img


def card(w, h, lines, big=0):
    """Title / end card."""
    img = np.full((h, w, 3), 252, np.uint8)
    y = h // 2 - 14 * len(lines)
    for i, (txt, sc, col, mono) in enumerate(lines):
        font = (cv2.FONT_HERSHEY_DUPLEX if mono else
                cv2.FONT_HERSHEY_SIMPLEX)
        (tw, th), _ = cv2.getTextSize(txt, font, sc, 2)
        cv2.putText(img, txt, ((w - tw) // 2, y), font, sc, col, 2,
                    cv2.LINE_AA)
        y += int(th + 26)
    return img


def panel(frame, title, colour, n_det, v, lost, guard, occ_on, note=None):
    f = cv2.resize(frame, PANEL, interpolation=cv2.INTER_AREA)
    h, w = f.shape[:2]
    cv2.rectangle(f, (0, 0), (w, 30), (255, 255, 255), -1)
    cv2.putText(f, title, (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.56, colour,
                2, cv2.LINE_AA)
    cv2.rectangle(f, (0, h - 26), (w, h), (255, 255, 255), -1)
    cv2.putText(f, "features %d/6" % n_det, (10, h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                (200, 60, 20) if n_det < 6 else (40, 40, 40), 1, cv2.LINE_AA)
    cv2.putText(f, "|v| %5.2f" % v, (150, h - 8), cv2.FONT_HERSHEY_SIMPLEX,
                0.48, colour, 2 if v > 2 else 1, cv2.LINE_AA)
    if guard:
        cv2.putText(f, guard, (w - 128, h - 8), cv2.FONT_HERSHEY_SIMPLEX,
                    0.46, (20, 120, 20), 1, cv2.LINE_AA)
    if occ_on:
        cv2.rectangle(f, (0, 0), (w - 1, h - 1), (30, 80, 220), 3)
    if note:
        cv2.rectangle(f, (6, 34), (w - 6, 62), (255, 255, 255), -1)
        cv2.putText(f, note, (12, 53), cv2.FONT_HERSHEY_SIMPLEX, 0.46,
                    (20, 90, 210), 1, cv2.LINE_AA)
    if lost:
        cv2.putText(f, "TARGET LOST", (w // 2 - 108, h // 2 + 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.92, (40, 40, 220), 3,
                    cv2.LINE_AA)
    return f


def main():
    os.makedirs("docs", exist_ok=True)
    thr = calibrate_area(P_RING, z_range=(0.5, 1.0), lateral=0.1)

    runs = []
    for label, fn, col in [
        ("Fixed: partition always on", lambda **k: run_partitioned(
            rel_tau=TAU, **k), (30, 30, 200)),
        ("Switched: area guard", lambda **k: run_switched(
            rel_tau=TAU, rule="area_guard", guard_thresh=thr, **k),
         (20, 130, 20)),
    ]:
        rec = Rec(P_RING, OCC, EXPOSURE)
        lg = fn(P_o=P_RING, cTo_init=CI, cTo_star=CS, lam=LAM, dt=DT,
                steps=STEPS, min_features=2, feature_fn=rec)
        runs.append(dict(label=label, col=col, rec=rec, lg=lg,
                         v=np.linalg.norm(lg["v"], axis=1)))
        print("%-30s ran %d steps, peak |v| %.2f, final |e| %.2e"
              % (label, len(lg["t"]), runs[-1]["v"].max(), lg["err"][-1]))

    # --- the moment worth calling out: worst divergence while both alive ---
    vL, vR = runs[0]["v"], runs[1]["v"]
    alive = min(len(runs[0]["rec"].cTo) - 1, WIN[1])
    ratios = {k: vL[k] / max(vR[k], 1e-9)
              for k in range(WIN[0], alive) if k < len(vL) and k < len(vR)}
    kcall = max(ratios, key=ratios.get)
    print("callout: step %d (t=%.2fs) left %.2f vs right %.2f = %.0fx"
          % (kcall, kcall * DT, vL[kcall], vR[kcall], ratios[kcall]))

    # --- display schedule: (step, fraction) ---
    sched = []
    for k in range(WIN[0], WIN[1]):
        sub = SUB_SPIKE if abs(k - kcall) <= 1 else SUB_SLOW   # half speed
        sched += [(k, j / sub) for j in range(sub)]
        if k == kcall:
            sched += [(k, 0.999)] * HOLD                       # and hold
    sched += [(k, j / SUB_TAIL) for k in range(WIN[1], TAIL_TO)
              for j in range(SUB_TAIL)]

    W = PANEL[0] * 2 + 6
    H = 34 + PANEL[1] + TRACE_H
    writer = imageio.get_writer(OUT, fps=FPS, quality=8, macro_block_size=8)
    frames = []

    title = card(W, H, [
        ("Guard the features your controller is built on,", 1.05,
         (20, 20, 20), False),
        ("not the matrix it inverts", 1.05, (20, 20, 20), False),
        ("same scene, same disruption, two controllers", 0.72,
         (110, 110, 110), False)])
    frames += [title] * CARD

    for k, a in sched:
        cols = []
        for r in runs:
            rec, v = r["rec"], r["v"]
            n = len(rec.cTo)
            lost = k >= n - 1
            i = min(k, n - 2) if n >= 2 else 0
            pose = interp(rec.cTo[i], rec.cTo[i + 1], a) if n >= 2 \
                else rec.cTo[0]
            rec._cur = rec.occl if k >= OCC_FROM else rec.clear
            img = rec._cur.render(pose)
            vv = float(v[min(k, len(v) - 1)]) if not lost else 0.0
            nd = rec.ndet[min(k, len(rec.ndet) - 1)]
            guard = ("guard: DROP" if r is runs[1] and nd < 3 else
                     ("guard: keep" if r is runs[1] else None))
            note = None
            if OCC_FROM <= k < OCC_FROM + 3:
                note = "orange slabs = occluder, covering 4 of 6 markers"
            p_img = panel(img, r["label"], r["col"], nd, vv, lost, guard,
                          k >= OCC_FROM, note)
            upto = min(k, len(v)) + 1
            tr = trace_strip(PANEL[0], list(v[:TAIL_TO]), upto, r["col"],
                             "commanded |v|, log scale", TAIL_TO)
            cols.append(np.vstack([p_img, tr]))
        gap = np.full((cols[0].shape[0], 6, 3), 255, np.uint8)
        row = np.hstack([cols[0], gap, cols[1]])
        bar = np.full((34, W, 3), 255, np.uint8)
        t_s = (k + a) * DT
        if abs(k - kcall) <= 1:
            msg = ("t=%4.2fs   LEFT |v| %.2f   vs   RIGHT |v| %.2f   = %.0fx"
                   % (t_s, vL[kcall], vR[kcall], ratios[kcall]))
            col = (30, 30, 200)
        elif k >= OCC_FROM:
            msg = "t=%4.2fs   occluder in - only 2 of 6 markers visible" % t_s
            col = (30, 80, 220)
        else:
            msg = "t=%4.2fs   all 6 markers visible" % t_s
            col = (30, 30, 30)
        cv2.putText(bar, msg, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.60, col,
                    2 if abs(k - kcall) <= 1 else 1, cv2.LINE_AA)
        frames.append(np.vstack([bar, row]))

    end = card(W, H, [
        ("Fixed partition: lost the target in %.1f s"
         % ((len(runs[0]["rec"].cTo) - 1) * DT), 0.92, (30, 30, 200), False),
        ("Feature guard: converged", 0.92, (20, 130, 20), False),
        ("one hand-written rule, no learned policy", 0.66, (110, 110, 110),
         False),
        (REPO, 0.78, (20, 20, 20), True)])
    frames += [end] * CARD

    for f in frames:
        writer.append_data(f)
    writer.close()
    for r in runs:
        r["rec"].close()

    # --- GIF: GitHub autoplays this inline; the mp4 needs a click ---
    # Shorter cards and a subsampled body, so the loop stays ~13 s and a
    # few MB while the mp4 keeps the full timing.
    gw = 600
    gh = int(round(H * gw / W / 2) * 2)
    body = frames[CARD:len(frames) - CARD]
    picked = frames[:12] + body[::3] + frames[-CARD:][:16]
    gif = [cv2.resize(f, (gw, gh), interpolation=cv2.INTER_AREA)
           for f in picked]
    imageio.mimsave(os.path.join("docs", "guard_vs_fixed.gif"), gif,
                    duration=1.0 / 9, loop=0)
    print("wrote %s  (%d frames, %.1f s at %d fps)"
          % (OUT, len(frames), len(frames) / FPS, FPS))
    print("wrote docs/guard_vs_fixed.gif  (%d frames, %.1f s)"
          % (len(gif), len(gif) / 9.0))


if __name__ == "__main__":
    main()
