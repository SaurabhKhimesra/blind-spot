# 14. The 2001 partition

## In plain words

Camera retreat happens because the solver gets to choose *how* to reduce the
image error, and it chooses badly. Corke and Hutchinson's fix in 2001: take
that choice away for the two motions that cause the trouble, and measure them
straight off the picture instead.

- **How far away am I?** Look at the size of the patch the markers cover.
- **How twisted am I?** Look at the angle of a line between two chosen
  markers.

The other four motions are still solved by the maths, but now with four
unknowns instead of six. Retreat disappears, because backing away is no longer
something the solver can invent — the distance term is measured.

## The two measured degrees of freedom

```python
sigma = sqrt(polygon area of the markers in the image)
alpha = angle of the line from marker 0 to marker 1

vz = lam_z * log(sigma_star / sigma)
wz = lam_a * wrap(alpha - alpha_star)
```

### Why the square root, and why the logarithm

This is the elegant part of the 2001 scheme and it is worth deriving.

A planar target seen face-on at depth `Z` projects to a polygon whose area
scales as `1/Z²` (everything divides by depth, chapter 3, and area is two
dimensions of it). Take the square root and you get a quantity that scales as
`1/Z`:

```
sigma ∝ 1/Z        so        sigma_star / sigma = Z / Z_star
```

The ratio of the measured feature to its desired value **is** the ratio of the
depths — with no need to know either one. Take the logarithm and you have a
depth error in log space:

```
log(sigma_star / sigma) = log Z − log Z_star
```

Driving `vz` proportionally to that is a proportional controller on the
*logarithm* of depth. Two consequences: it is scale-free, so a 2× error far
away and a 2× error close up produce corrections of the same size in log
terms; and it approaches smoothly without overshoot, because the error shrinks
as the camera closes in.

And the entire thing is computed from the picture. No depth sensor, no target
model, no calibration of distance. That is why the scheme is attractive, and
**it is also the vulnerability this whole book is about**: `sigma` stands in
for depth only as long as nothing else changes the patch size. Tilt the part
(chapter 25) or fold it (chapter 26) and the substitute lies.

### Why the angle, and the sign

Rolling the camera by `ωz` rotates the whole image by exactly the opposite
amount, so `d(alpha)/d(ωz) = −1` exactly. That makes the correct sign in the
law positive. It is one line of code and it is the kind of thing that is 50/50
if you reason it out, so it was checked by finite difference instead — nudge
`ωz`, watch the measured angle, compare. Getting it backwards makes the camera
rotate away from the goal, which looks like a control problem and is not.

The pair of markers used for `alpha` must be **fixed in advance**, and the
same pair in the current and desired image. Choosing it from the current
picture — say the longest visible line — makes `alpha` jump every time the
choice changes, which on a symmetric target happens constantly.

## The remaining four degrees of freedom

Split the interaction matrix by columns: `L_z` for `vz, ωz` and `L_xy` for
`vx, vy, ωx, ωy`. Then

```
ṡ = L_xy · v_xy + L_z · v_z
```

We want `ṡ = −λ e`, and `v_z` is already decided, so

```
v_xy = −pinv(L_xy) · (λ e + L_z v_z)
```

The term `L_z v_z` is the **coupling term**, and it is **added**. The reason is
physical: `vz` and `ωz` are about to move every marker in the image, so the
other four degrees of freedom must cancel that motion *in addition to* driving
the error down. Subtracting it instead makes the controller fight itself, and
that was one of the two sign bugs finite differences caught.

## Why retreat dies

Retreat requires the solver to choose backing away. In this law, the camera's
depth motion comes from one measurement — is the patch smaller than it should
be? — and nothing else can ask for it. So the pathology has nowhere to live.

| controller | retreat |
|---|---|
| classic IBVS | 68.19 m, diverges |
| partitioned (2001) | **0.80 m, converges** |

0.80 m is where it started. It never backed up.

## What it costs

**Two features left**: the reduced solve becomes an exactly determined 4×4
with no slack, and the spike is **288.4×** (chapter 12).

**Collapsed target**: four degrees of freedom are forced through an
ill-conditioned `L_xy` that truncation is no longer free to prune, and the run
floors at **7.3e-3** (chapter 13).

**Clean cost**: on an undisturbed run, **+13%** more steps to drive the error
below 1% of its initial value. Worth quoting because a fix with no measured
downside is usually a fix nobody measured.

## The idea that follows

A law that is right in some situations and wrong in others should not be a
design-time choice. It should be a runtime one — which is chapter 16, and the
only remaining question is what to switch on.

## Say it like this

> The 2001 partition removes camera retreat by driving two degrees of freedom
> from direct image measurements instead of from the inverse: the square root
> of the marker polygon's area, which scales as one over depth so its log is a
> depth error you never needed a depth sensor for, and the angle of a line
> between two fixed markers. It genuinely fixes retreat. It also makes the
> remaining solve exactly determined, which is why it spikes 288× on two
> features and can't converge on a collapsed target.
