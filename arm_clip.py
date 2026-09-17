"""Assemble the folding-part clip from one recorded Gazebo run.

    RECORD=1 RATE=120 OUT=~/blindspot_clip/take1 ./run_fold_demo.sh
    .venv/bin/python arm_clip.py ~/blindspot_clip/take1

Every frame of robot motion in the clip is a frame Gazebo rendered during that
run, and every number on screen is read from that run's log. The edit changes
only timing (speed-up, slow motion, one rewind), crops and overlays.
"""

import csv
import os
import sys

import cv2
import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

RUN = os.path.expanduser(sys.argv[1])
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(RUN, "blindspot_fold.mp4")
W, H, FPS = 1920, 1080, 30
FOLD_MAX = np.deg2rad(85.0)
STOP_STANDOFF = 0.12

BG = (13, 15, 19)
PANEL = (24, 27, 33)
RED = (255, 84, 84)
GREEN = (60, 214, 150)
AMBER = (255, 178, 44)
WHITE = (241, 243, 247)
GREY = (140, 147, 160)

_FB = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
_FR = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
_FM = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"
_fonts = {}


def font(size, kind="b"):
    key = (size, kind)
    if key not in _fonts:
        if kind == "m":
            _fonts[key] = ImageFont.truetype(_FM, size)
        else:
            _fonts[key] = ImageFont.truetype(_FB if kind == "b" else _FR, size,
                                             index=0)
    return _fonts[key]


# ---------------------------------------------------------------- the run
class Frames:
    """Frames named by sim timestamp; a camera shows its latest capture."""

    def __init__(self, d):
        self.dir = d
        self.files = sorted(os.listdir(d))
        self.t = np.array([int(f[:-4]) * 1e-9 for f in self.files])
        self.cache = {}

    def at(self, sim):
        i = int(np.searchsorted(self.t, sim, side="right")) - 1
        i = min(max(i, 0), len(self.files) - 1)
        if i not in self.cache:
            if len(self.cache) > 96:
                self.cache.clear()
            im = cv2.imread(os.path.join(self.dir, self.files[i]))
            self.cache[i] = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
        return self.cache[i]


rows = list(csv.DictReader(open(os.path.join(RUN, "log.csv"))))
T0 = float(rows[0]["sim"]) - float(rows[0]["t"])       # act time 0 in sim time


def col(side, key):
    R = [r for r in rows if r["side"] == side]
    t = np.array([float(r["t"]) for r in R])
    v = np.array([float(r[key]) if r[key] != "" else np.nan for r in R])
    return t, v


tL, dL = col("left", "standoff")
tR, dR = col("right", "standoff")
_, mR = col("right", "margin")
_, pR = col("right", "partition")
_, nL = col("left", "n_feat")
_, nR = col("right", "n_feat")
_, fold = col("left", "fold")
_, stopL = col("left", "stopped_at")
_, stopR = col("right", "stopped_at")

i_fold = int(np.argmax(fold > 0))
T_FOLD = tL[i_fold] - fold[i_fold] / FOLD_MAX           # fold ramp is 1 s long
T_STOP = float(np.nanmin(stopL)) if np.any(np.isfinite(stopL)) else None
T_STOP_R = float(np.nanmin(stopR)) if np.any(np.isfinite(stopR)) else None
_after = (tR >= T_FOLD) & (pR == 0)
T_GUARD = float(tR[np.argmax(_after)]) if _after.any() else None
T_END = float(tL[-1])


def step(t, v, tau):
    i = int(np.searchsorted(t, tau, side="right")) - 1
    return v[max(i, 0)]


heroL = Frames(os.path.join(RUN, "frames", "cine_left"))
heroR = Frames(os.path.join(RUN, "frames", "cine_right"))
wristL = Frames(os.path.join(RUN, "frames", "wrist_left"))
wristR = Frames(os.path.join(RUN, "frames", "wrist_right"))


# ------------------------------------------------------ perception overlay
def detect_dots(rgb):
    """Identical to the node's detector, so the overlay shows what it saw."""
    g = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    _, th = cv2.threshold(g, 90, 255, cv2.THRESH_BINARY_INV)
    n, _, stats, cent = cv2.connectedComponentsWithStats(th, 8)
    cand = [(int(s[4]), c) for s, c in zip(stats[1:], cent[1:])
            if 25 < s[4] < 4000]
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


def order_by_angle(p):
    c = p.mean(axis=0)
    return p[np.argsort(np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0]))]


# ------------------------------------------------------------ drawing kit
def paste(canvas, arr, x, y, w, h):
    canvas.paste(Image.fromarray(cv2.resize(arr, (w, h),
                                            interpolation=cv2.INTER_AREA)),
                 (x, y))


def text(d, xy, s, size, fill=WHITE, kind="b", anchor="la"):
    d.text(xy, s, font=font(size, kind), fill=fill, anchor=anchor)


def chip(d, x, y, s, size, fg, bg, pad=12, anchor="l"):
    f = font(size, "b")
    x0, y0, x1, y1 = d.textbbox((0, 0), s, font=f)
    w, h = x1 - x0 + 2 * pad, size + 2 * pad - 4
    if anchor == "c":
        x -= w // 2
    elif anchor == "r":
        x -= w
    d.rounded_rectangle([x, y, x + w, y + h], radius=h // 2, fill=bg)
    d.text((x + pad - x0, y + (h - size) // 2 - 2), s, font=f, fill=fg)
    return w


def caption(canvas, s, y=560, size=40, alpha=1.0):
    if not s or alpha <= 0:
        return
    ov = Image.new("RGBA", canvas.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    f = font(size, "b")
    x0, y0, x1, y1 = d.textbbox((0, 0), s, font=f)
    w = x1 - x0 + 56
    x = (W - w) // 2
    d.rounded_rectangle([x, y - 34, x + w, y + 34],
                        radius=16, fill=(8, 9, 12, int(215 * alpha)))
    d.text((W // 2, y), s, font=f, fill=WHITE + (int(255 * alpha),),
           anchor="mm")
    canvas.paste(Image.alpha_composite(canvas.convert("RGBA"), ov)
                 .convert("RGB"))


def wrist_panel(side, tau, w, h, flash=0.0):
    frames = wristL if side == "left" else wristR
    img = frames.at(T0 + tau).copy()
    uv = detect_dots(img)
    guarded = side == "right"
    part = bool(step(tR, pR, tau)) if guarded else True
    stopped = (T_STOP is not None and side == "left" and tau >= T_STOP)
    col = (GREEN if part else AMBER) if guarded else (RED if stopped else WHITE)
    if len(uv) >= 3:
        o = order_by_angle(uv).astype(np.int32)
        over = img.copy()
        cv2.fillPoly(over, [o], col)
        img = cv2.addWeighted(over, 0.28, img, 0.72, 0)
        cv2.polylines(img, [o], True, col, 2, cv2.LINE_AA)
        for u, v in o:
            cv2.circle(img, (int(u), int(v)), 9, col, 2, cv2.LINE_AA)
    im = Image.fromarray(cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA))
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, w, 44], fill=(10, 11, 14))
    text(d, (14, 8), "WRIST CAMERA", 22, GREY)
    n = int(step(tL if side == "left" else tR, nL if side == "left" else nR, tau))
    text(d, (w - 14, 8), "markers %d/6" % n, 22, WHITE, anchor="ra")
    if guarded:
        m = float(step(tR, mR, tau))
        d.rectangle([0, h - 92, w, h], fill=(10, 11, 14))
        text(d, (14, h - 84), "GUARD  pattern area vs healthy", 22, GREY)
        x0, x1, yb = 16, w - 150, h - 34
        d.rounded_rectangle([x0, yb - 12, x1, yb + 12], radius=12, fill=(40, 44, 52))
        frac = float(np.clip(m / 2.0, 0, 1))
        d.rounded_rectangle([x0, yb - 12, x0 + max(24, int((x1 - x0) * frac)), yb + 12],
                            radius=12, fill=GREEN if m >= 1 else AMBER)
        xt = x0 + int((x1 - x0) * 0.5)
        d.line([xt, yb - 20, xt, yb + 20], fill=WHITE, width=3)
        text(d, (w - 16, yb), "%.2fx" % m, 30, GREEN if m >= 1 else AMBER,
             kind="m", anchor="rm")
    else:
        d.rectangle([0, h - 92, w, h], fill=(10, 11, 14))
        text(d, (14, h - 84), "NO GUARD", 22, GREY)
        text(d, (14, h - 50), "partition always on", 30, WHITE)
    if flash > 0:
        ov = Image.new("RGB", im.size, col)
        im = Image.blend(im, ov, 0.35 * flash)
    return im


def graph(d, x, y, w, h, tau, t_lo, t_hi, big=False):
    """Distance to the part, both arms, with the protective-stop line."""
    lo, hi = 0.08, 0.30

    def px(t, v):
        return (x + (t - t_lo) / (t_hi - t_lo) * w,
                y + h - (v - lo) / (hi - lo) * h)

    d.rectangle([x, y, x + w, y + h], fill=(18, 20, 25))
    for cm in (10, 15, 20, 25, 30):
        _, yy = px(t_lo, cm / 100)
        d.line([x, yy, x + w, yy], fill=(34, 37, 44), width=1)
        text(d, (x - 10, yy), "%d" % cm, 20 if not big else 26, GREY,
             kind="m", anchor="rm")
    if big:
        text(d, (x - 10, y - 26), "cm", 26, GREY, "r", "rm")
        for dt in range(0, int(t_hi - T_FOLD) + 1):
            xx, _ = px(T_FOLD + dt, lo)
            if xx <= x + w:
                d.line([xx, y + h, xx, y + h + 10], fill=(70, 75, 86), width=2)
                text(d, (xx, y + h + 18), "+%d s" % dt if dt else "fold",
                     26, GREY, "r", "ma")
    _, ys = px(t_lo, STOP_STANDOFF)
    for xx in range(int(x), int(x + w), 18):
        d.line([xx, ys, min(xx + 9, x + w), ys], fill=RED, width=2)
    text(d, (x + w - 8, ys - 6), "protective stop", 20 if not big else 26,
         RED, anchor="rb")
    if t_lo <= T_FOLD <= t_hi:
        xf, _ = px(T_FOLD, lo)
        d.line([xf, y, xf, y + h], fill=AMBER, width=2)
        text(d, (xf + 8, y + 6), "part folds", 20 if not big else 26, AMBER)
    for t, v, c in ((tL, dL, RED), (tR, dR, GREEN)):
        sel = (t >= t_lo) & (t <= min(tau, t_hi))
        pts = [px(tt, vv) for tt, vv in zip(t[sel], v[sel])]
        if len(pts) >= 2:
            d.line(pts, fill=c, width=5 if big else 4, joint="curve")
        if pts:
            d.ellipse([pts[-1][0] - 7, pts[-1][1] - 7, pts[-1][0] + 7,
                       pts[-1][1] + 7], fill=c)


def hud(tau, cap=None, cap_alpha=1.0, speed=None):
    """The main split-screen layout at act time tau."""
    c = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(c)
    # headers
    d.rectangle([0, 0, W, 70], fill=(10, 11, 14))
    wl = chip(d, 24, 14, "WITHOUT GUARD", 26, (10, 11, 14), RED)
    text(d, (24 + wl + 18, 35), "2001 partitioned visual servoing", 26, WHITE,
         "r", "lm")
    wr = chip(d, W - 24, 14, "WITH GUARD", 26, (10, 11, 14), GREEN, anchor="r")
    text(d, (W - 24 - wr - 18, 35), "same controller + feature guard", 26,
         WHITE, "r", "rm")
    # hero shots
    stop_age = None if T_STOP is None else tau - T_STOP
    guard_age = None if T_GUARD is None else tau - T_GUARD
    paste(c, heroL.at(T0 + tau), 0, 70, 958, 540)
    paste(c, heroR.at(T0 + tau), 962, 70, 958, 540)
    d.rectangle([958, 70, 962, 610], fill=BG)
    if stop_age is not None and stop_age >= 0:
        pulse = max(0.0, 1.0 - stop_age / 0.6)
        wdt = 6 + int(10 * pulse)
        d.rectangle([0, 70, 957, 609], outline=RED, width=wdt)
        chip(d, 479, 100, "PROTECTIVE STOP", 44 + int(10 * pulse),
             WHITE, RED, pad=18, anchor="c")
    if guard_age is not None and guard_age >= 0:
        pulse = max(0.0, 1.0 - guard_age / 0.6)
        d.rectangle([963, 70, W - 1, 609], outline=AMBER, width=4 + int(8 * pulse))
        chip(d, 962 + 479, 100, "GUARD: PARTITION OFF", 40 + int(8 * pulse),
             (10, 11, 14), AMBER, pad=16, anchor="c")
    if speed:
        chip(d, 16, 566, speed, 24, WHITE, (10, 11, 14))
    # bottom row
    c.paste(wrist_panel("left", tau, 627, 470,
                        flash=0 if stop_age is None or stop_age < 0
                        else max(0, 1 - stop_age / 0.5)), (0, 610))
    c.paste(wrist_panel("right", tau, 627, 470,
                        flash=0 if guard_age is None or guard_age < 0
                        else max(0, 1 - guard_age / 0.5)), (1293, 610))
    d.rectangle([627, 610, 1293, H], fill=PANEL)
    text(d, (960, 640), "DISTANCE TO THE PART", 24, GREY, anchor="mm")
    vl = float(np.interp(tau, tL, dL)) * 100
    vr = float(np.interp(tau, tR, dR)) * 100
    text(d, (790, 700), "%.1f cm" % vl, 50, RED, "m", "mm")
    text(d, (1130, 700), "%.1f cm" % vr, 50, GREEN, "m", "mm")
    graph(d, 700, 745, 570, 300, tau, max(8.0, tau - 8.0), max(16.0, tau))
    if cap:
        caption(c, cap, y=560, alpha=cap_alpha)
    return c


# ----------------------------------------------------------------- segments
def ease(x):
    x = float(np.clip(x, 0, 1))
    return x * x * (3 - 2 * x)


def title_card(lines, base=None, dim=0.4):
    c = Image.new("RGB", (W, H), BG)
    if base is not None:
        arr = cv2.GaussianBlur(np.asarray(base), (0, 0), 18)
        c = Image.fromarray((arr * dim).astype(np.uint8))
    d = ImageDraw.Draw(c)
    y = H // 2 - sum(l[1] + l[3] for l in lines) // 2
    for s, size, colr, gap, kind in lines:
        text(d, (W // 2, y + size // 2), s, size, colr, kind, "mm")
        y += size + gap
    return c


def main():
    frames = []

    def add(img, n=1):
        a = np.asarray(img.convert("RGB"))
        for _ in range(n):
            frames.append(a)

    print("act: fold %.2f s, guard %.2f s, protective stop %s, end %.2f s"
          % (T_FOLD, T_GUARD, T_STOP, T_END))
    if T_STOP_R is not None:
        raise SystemExit("the GUARDED cell tripped its stop - this take "
                         "does not show what the clip claims; do not cut it")

    # 1. hook: the moment itself, slow, no explanation
    t_a, t_b = T_FOLD + 0.1, min(T_END, (T_STOP or T_FOLD + 1.4) + 1.0)
    n = int((t_b - t_a) / 0.5 * FPS)
    for k in range(n):
        tau = t_a + (t_b - t_a) * k / (n - 1)
        img = hud(tau, cap="Same robot. Same camera. Same part.",
                  cap_alpha=1.0, speed="0.5x")
        add(img)
    add(Image.fromarray(frames[-1]), 12)
    # rewind
    for k in range(24):
        tau = t_b - (t_b - 10.0) * ease(k / 23)
        img = hud(tau)
        arr = np.asarray(img).astype(np.float32)
        arr = (0.55 * arr + 0.45 * arr.mean(axis=2, keepdims=True)) * 0.8
        img = Image.fromarray(arr.clip(0, 255).astype(np.uint8))
        chip(ImageDraw.Draw(img), W // 2, H // 2 - 50, "REWIND", 60, WHITE,
             (10, 11, 14), pad=26, anchor="c")
        add(img)

    # 2. title
    base = hud(17.0)
    card = title_card([
        ("THE BLIND SPOT", 96, WHITE, 26, "b"),
        ("Two identical robot arms. A part that folds.", 44, GREY, 18, "r"),
        ("One of them checks the geometry its controller relies on.", 44, GREY, 0, "r"),
    ], base=base)
    add(card, int(3.2 * FPS))

    # 3. setup, sped up
    caps = [(9.0, 12.5, "Both arms servo to the part using six markers on their wrist camera."),
            (12.5, 16.0, "Left: the 2001 partitioned controller. Right: the same controller, plus the guard."),
            (16.0, T_FOLD - 0.3, "Identical code. Identical motion.")]
    t_a, t_b, sp = 9.0, T_FOLD - 0.3, 1.6
    n = int((t_b - t_a) / sp * FPS)
    for k in range(n):
        tau = t_a + (t_b - t_a) * k / (n - 1)
        cap = next((s for a, b, s in caps if a <= tau < b), None)
        add(hud(tau, cap=cap, speed="%.1fx" % sp))

    # 4. the fold, slow
    caps = [(T_FOLD - 0.3, T_GUARD or T_FOLD + 1.0,
             "The part folds. All six markers stay in view."),
            (T_GUARD or 99, T_STOP or 99,
             "Right: the marker pattern collapsed, so the guard turns the partition off."),
            (T_STOP or 99, T_END + 1,
             "Left: the partition reads the shrinking pattern as distance and drives at the part.")]
    t_a, t_mid, t_b = T_FOLD - 0.3, min(T_END, (T_STOP or T_END) + 0.8), T_END
    for (a, b, sp) in ((t_a, t_mid, 0.4), (t_mid, t_b, 1.0)):
        n = max(2, int((b - a) / sp * FPS))
        for k in range(n):
            tau = a + (b - a) * k / (n - 1)
            cap = next((s for lo, hi, s in caps if lo <= tau < hi), None)
            add(hud(tau, cap=cap, speed="%.1fx" % sp))
    add(Image.fromarray(frames[-1]), int(1.0 * FPS))

    # 5. the numbers, full screen
    t_lo, t_hi = 18.5, T_END
    for k in range(int(4.5 * FPS)):
        tau = t_lo + (t_hi - t_lo) * ease(k / (3.0 * FPS))
        c = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(c)
        text(d, (120, 90), "Distance from wrist camera to the part", 48, WHITE)
        text(d, (120, 150), "logged by the simulation, both arms", 30, GREY, "r")
        graph(d, 190, 230, 1540, 700, tau, t_lo, t_hi, big=True)
        text(d, (190, 1000), "time from the moment the part starts folding",
             26, GREY, "r")
        if tau >= (T_STOP or 99):
            text(d, (1730, 250), "without guard: %.0f cm -> %.0f cm, protective stop"
                 % (dL[tL <= T_FOLD][-1] * 100, np.interp(T_STOP, tL, dL) * 100),
                 34, RED, anchor="ra")
        if tau >= (T_GUARD or 99):
            text(d, (1730, 300), "with guard: switched at %.1f s, stayed at %.0f cm"
                 % (T_GUARD - T_FOLD, dR[-1] * 100), 34, GREEN, anchor="ra")
        add(c)

    # 6. end cards
    add(title_card([
        ("Why it happens", 64, WHITE, 36, "b"),
        ("The 2001 partitioned controller estimates distance", 44, GREY, 12, "r"),
        ("from the area of the marker pattern.", 44, GREY, 36, "r"),
        ("A folding part shrinks that area, so it drives at the part.", 44, WHITE, 12, "b"),
        ("The guard checks that area against a calibrated healthy range.", 44, WHITE, 0, "b"),
    ]), int(4.8 * FPS))
    add(title_card([
        ("Gazebo + ROS 2 simulation  ·  UR5e  ·  10 Hz visual servo loop", 40, GREY, 24, "r"),
        ("The guard catches a fold that outpaces the controller.", 40, WHITE, 10, "r"),
        ("Slower folds, and what happens after, are documented limits.", 40, WHITE, 44, "r"),
        ("github.com/SaurabhKhimesra/blind-spot", 56, GREEN, 0, "b"),
    ]), int(4.5 * FPS))

    print("writing %d frames (%.1f s) to %s" % (len(frames), len(frames) / FPS, OUT))
    wr = imageio.get_writer(OUT, fps=FPS, codec="libx264", quality=None,
                            macro_block_size=1, pixelformat="yuv420p",
                            ffmpeg_params=["-crf", "18", "-preset", "slow",
                                           "-movflags", "+faststart"])
    for f in frames:
        wr.append_data(f)
    wr.close()


if __name__ == "__main__":
    main()
