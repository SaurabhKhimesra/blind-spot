# 4. The 2001 partition

## In plain words

The basic method has one big weakness: ask the robot to *spin* the camera
halfway round and it backs away instead. Corke and Hutchinson's idea in 2001
was simple and clever. Instead of asking the maths to work out all six ways
the camera can move, take the two awkward ones out and measure them directly
from the picture:

- **How far away am I?** Look at how big the patch covered by the markers is.
  Big patch, I'm close. Small patch, I'm far.
- **How twisted am I?** Look at the angle of a line drawn between two chosen
  markers, and compare it to the angle it should have.

The other four ways of moving are still worked out by the maths, but now they
only have four unknowns to solve instead of six. The spinning problem
disappears, because spinning is no longer something the solver has to guess
at; it is measured.

The catch, which is the whole project: those two measurements are *substitutes*
for the real quantities. A substitute can be wrong without saying so.

## The law, precisely

Two degrees of freedom are driven from image measurements
(`partitioned.py`):

```python
sigma = sqrt(area of the feature polygon)      # substitute for depth
alpha = angle of the line between feature 0 and feature 1

vz = lam_z * log(sigma_star / sigma)           # push in or pull out
wz = lam_a * wrap(alpha - alpha_star)          # roll about the optical axis
```

The remaining four (`vx, vy, ωx, ωy`) come from a *reduced* interaction
matrix: `L_xy`, the 2N×4 matrix of the columns for those four DOF.

```python
v_xy = -pinv(L_xy) @ (lam * e + L_z @ [vz, wz])
```

The term `L_z @ [vz, wz]` is the **coupling term**, and it is added, not
subtracted. Reason: `vz` and `wz` are about to move every feature in the
image. The other four DOF must cancel that motion on top of driving the error
down. Get the sign wrong and the controller fights itself.

Three details that are easy to get wrong, and were:

- **The sign of `wz`.** `d(alpha)/d(wz) = −1` exactly, so the sign in the law
  is positive. The comment in `partitioned.py` says "verified numerically"
  because it was: by finite difference, not by reasoning. Getting it backwards
  makes the camera rotate away from the goal.
- **The feature pair for alpha must be fixed.** Choosing it from the current
  image — say, the longest line — makes alpha jump every time the choice
  changes, which on a symmetric target happens constantly.
- **The references must be recomputed from the same visible subset.** If three
  features are occluded, `sigma*` and `alpha*` are recomputed from the three
  that remain. Otherwise the comparison is meaningless and the controller is a
  strawman.

## Why it kills camera retreat

Retreat happens because the least-squares solver decides that backing away is
a cheap way to make the image error decay. Once `vz` is not produced by the
solver at all — it comes from the area measurement — that option is gone. The
camera cannot retreat, because nothing in the law can ask it to, except the
area saying "you are too far".

Measured: classic IBVS 68.19 m and diverging; partitioned **0.80 m and
converging**. The fix is real and it is not in dispute.

## What it costs

**Two features left.** Removing two columns from the inverse turns a
comfortably overdetermined problem into an exactly determined 4×4 one. With
two features, `L_xy` is 4×4 with no slack left. Classic IBVS, in the same
situation, is *underdetermined* and gets the minimum-norm solution, which is
small. Measured spike: classic **0.9×**, partitioned **288.4×**.

**Collapsed target.** All six features visible, squashed toward a line. The
reduced matrix is `[3.68, 3.66, 0.0124, 0.0037]`. The partition forces four
DOF through it. Truncation would normally decline to move along the weak
directions, but the partition has already committed those DOF to the solve.
The run floors at **7.3e-3** instead of converging.

**Clean cost.** On an undegraded run it is **+13%** more steps to drive the
error below 1% of its initial value. Worth stating because "a fix with no
downside" is usually a fix that has not been measured. That number is also
threshold-dependent: +2.4% at a 50% threshold, +7.4% at 0.01%. Quoting it
without the threshold is meaningless, which is why the README defines it in
the caption.

## The idea that follows

If the 2001 partition is the right law in some situations and the wrong law in
others, then it should not be a design-time decision at all. It should be a
**runtime** decision. That is the switch, and the only question left is what
to switch on.

## Say it like this

> The 2001 fix takes the two degrees of freedom that cause camera retreat out
> of the inverse and drives them from direct image measurements: the area of
> the marker polygon stands in for distance, and the angle of a line between
> two markers stands in for roll. It genuinely fixes retreat — 68 metres down
> to 0.8. But it makes the remaining solve exactly determined, so when
> features drop to two it spikes nearly 300×, and when the target's geometry
> collapses it can't converge. So the real question isn't which law is better,
> it's when to run which.
