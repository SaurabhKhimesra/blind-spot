# 2. Visual servoing from scratch

## In plain words

Imagine guiding your hand to a doorknob with your eyes. You don't measure the
distance in centimetres; you look at where the knob is in your view, notice
it's too far left, and move right until it's centred. Then you do it again.

That is **visual servoing**. The robot has a camera, the camera sees some
marks on the part, and the controller keeps asking one question: *which way
should I move so those marks slide toward where they should be?*

To answer it, you need a rule connecting "how the robot moves" to "how the
marks slide in the picture". That rule is a table of numbers called the
**interaction matrix**. The controller works out the motion it wants in the
picture, then effectively runs that table backwards to find the robot motion
that produces it. Running a table backwards is called inverting it, and
**everything that goes wrong in this project goes wrong inside that
inversion**.

The rest of this chapter is the same story with the maths written down.

## The setup

A camera is mounted on the robot's wrist (**eye in hand**). The target carries
`N` point features. The controller's job is to move the camera until what it
sees matches what it should see when the robot is in the right place.

Two ways to do that:

- **Position-based (PBVS)**: estimate the full 3D pose from the image, then
  drive the pose error to zero. Needs a pose estimate, which needs a model and
  good calibration, and the image can leave the frame while the pose error
  looks fine.
- **Image-based (IBVS)**: work directly in the image. No pose estimate. The
  error is defined in pixels, so features stay in frame naturally. The price
  is that the relationship between camera motion and image motion is
  nonlinear and depends on depth.

This project is entirely IBVS, because that is where the interesting failure
modes live.

## Features and error

A 3D point in the camera frame `(X, Y, Z)` projects to normalised image
coordinates

```
x = X / Z,      y = Y / Z
```

Normalised means the intrinsics have already been divided out: pixels
converted with `x = (u − cx) / fx`. Everything in the repo works in these
units, which is why `blindspot/units.py` exists and why the ROS node insists
on `camera_info`.

Stack the current features into `s` and the desired ones into `s*`. The error
is simply

```
e = s − s*          (2N values)
```

## The interaction matrix

The whole method comes from one question: *if the camera moves with twist
`v = (vx, vy, vz, ωx, ωy, ωz)`, how fast does each feature move in the image?*
Differentiating the projection gives, per point, two rows:

```
ẋ = [ −1/Z,   0,   x/Z,    x·y,   −(1 + x²),   y ] · v
ẏ = [   0,  −1/Z,  y/Z,  1 + y²,    −x·y,     −x ] · v
```

Stack them for all N points and you have the **interaction matrix** `L`, of
shape `2N × 6`, so that `ṡ = L v`. That is exactly `interaction_matrix()` in
`ibvs_core.py`, and every one of its entries is checked by finite difference
in `fd_check.py` rather than trusted from the page it was copied from.

Three things to notice, because all three become failures later:

1. **Depth appears only in the translation columns.** Rotation rows do not
   contain `Z`. You can control rotation without knowing depth; you cannot
   control translation without some estimate of it.
2. **The column for `vx` and the column for `ωy` look alike** when the target
   is small and far away: `−1/Z` against `−(1 + x²)`. Sliding sideways and
   rotating about the vertical axis produce nearly the same image motion. That
   near-ambiguity is a pair of very small singular values, and chapter 12 is
   the story of what happens when a controller's command lands in them.
3. **A planar target seen face-on is the worst-conditioned case**, because
   every point sits at the same depth, so depth motion and rotation are hard
   to tell apart. Tilting the target *improves* conditioning. This is
   counter-intuitive and it is the reason a signal based on the matrix and a
   signal based on the picture disagree (chapter 12).

## The control law

Ask for an exponential decay of the error, `ė = −λ e`. Since `ė = L v`:

```
v = −λ · L⁺ e
```

where `L⁺` is the pseudo-inverse (`np.linalg.pinv`, i.e. a least-squares
solve). With `2N ≥ 6` this is overdetermined and the pseudo-inverse takes the
least-squares solution; with fewer features it is underdetermined and takes
the minimum-norm one. That distinction matters in chapter 3: the
minimum-norm solution is what accidentally protects classic IBVS in the
two-feature case.

`λ` sets how aggressively the error is driven down. The study uses λ = 0.5,
and `dt = 0.033 s` (30 Hz) unless a section says otherwise.

## Integrating the motion

The commanded twist has to move the camera pose. Poses are 4×4 matrices `cTo`
(target expressed in the camera frame), and the update is the matrix
exponential of the twist:

```
cTo ← se3_exp(−v · dt) · cTo
```

`se3_exp` and its inverse `se3_log` are in `ibvs_core.py`, and `se3_log` is
verified by finite difference too — because a sign error there is invisible
until the controller behaves oddly in a way you will blame on the control law.

## Where depth comes from

`L` needs `Z` per point. Three honest options, all implemented
(`depth_mode` in `run_ibvs`):

- `true` — the exact current depth. Only available in simulation, used as the
  control condition.
- `desired` — the depth at the goal pose. Standard practice.
- `constant` — one number for all points.

The Gazebo demo uses a constant depth (the desired standoff, 0.26 m), which is
what you would do on a real cell with no depth sensor. That choice matters in
chapter 13: with constant depth, a folded target makes `L` rank-deficient
through a different mechanism than it does with true depths.

## The health of the matrix

Since the method inverts `L`, the natural question is "how invertible is it
right now?". The project's metric is **σ₆**: the sixth singular value of `L`,
with the spectrum padded to length 6 so that a matrix with fewer than six
rows reads exactly zero.

That padding is not a detail. An earlier version used `S[-1]`, the smallest
singular value of whatever shape `L` happened to be. On a two-feature `L`
(4×6) that reads a healthy-looking **9.97e-2** while the true sixth singular
value is **exactly 0** — the matrix has a null space and the metric could not
see it. Comparing matrices of different shapes was the bug; padding to a fixed
length is the fix.

## Say it like this

> IBVS drives the robot from image error directly. You build the interaction
> matrix, which maps camera velocity to feature velocity in the image, and
> then you invert it and ask for an exponential decay of the error. Everything
> that goes wrong in this project goes wrong inside that inverse: either
> because the matrix genuinely loses rank, or because the controller's command
> lands in a direction the image can barely see.
