# 13. Failure 4: the collapsed target

## In plain words

This is the one that should worry you.

Nothing is hidden. All six markers are visible, crisply detected, right where
the detector expects them. Every safety check based on "can I still see my
markers?" says yes, continuously, the whole time.

And the controller cannot finish the job, because the six markers have stopped
carrying the information it needs. Their geometry has flattened toward a line.

## Why a flat arrangement carries less information

Six points in a ring pin the camera down in every direction: move left and the
ring shifts, roll and it spins, approach and it grows.

Squash that ring toward a line and one of those is lost. Rotating about the
line barely changes the picture, because points *on* an axis of rotation do
not move when you rotate about it. The camera can move a long way with almost
no evidence appearing in the image — which is exactly the definition of a weak
direction from chapter 4.

In the benchmark, the four singular values of the matrix the partition must
invert come out as

```
L_xy = [ 3.68, 3.66, 0.0124, 0.0037 ]
```

Three orders of magnitude between the strongest and the weakest. Inverting
that multiplies anything landing in the weak direction by about 270.

## Why counting features cannot see it

The feature count is **6** at every step. A rule that watches the count
reports perfect health, permanently, while the controller fails.

This is the single strongest argument for the shipped guard, and it is why the
guard watches a *geometric* quantity rather than a count. It is also the case
the ROS demo shows explicitly: six markers present, guard correctly dropping
the partition (chapter 22).

## What each controller does

| controller | final image error | verdict |
|---|---|---|
| classic IBVS | 8e-11 | converges |
| adaptive-rank truncation | 2.4e-5 | converges |
| partitioned (2001) | 7.3e-3 | **floors, never reaches 1e-4** |
| both combined (fixed) | 2.0e-3 | **floors** |
| switched, feature count | 2.0e-3 | **floors** — counting cannot see it |
| switched, σ₆ or area guard | 2.4e-5 | converges |

Truncation converges because it is allowed to decline the blind direction. The
partition cannot, because it has already committed those degrees of freedom to
its reduced solve, so the blind direction gets inverted anyway.

## Two corrections I had to make here

Both were mine, both were wrong first, and both are in the repository next to
the result rather than quietly fixed.

**The mechanism is not "the area goes to zero".** The obvious story — the
polygon area shrinks, so the depth term `log(σ*/σ)` blows up — is false. The
*desired* area shrinks along with the current one, the ratio stays at
**1.4980**, and the commanded `vz` is unchanged at every collapse level. The
failure is in the reduced inverse, not the depth feature. I checked because the
story was too tidy.

**This case must be judged on convergence, not on its velocity spike.** At the
shipped τ = 1e-3, the truncation threshold happens to land at **98.78%** of the
smallest singular value — a 1.2% margin. So whether a direction is kept or
dropped flips on a coin, and the spike moves from **5.96** (τ=1e-4) to **0.33**
(τ=1e-2), where it matches truncation-only's 0.32. A number that moves by 18×
when you nudge a constant is not evidence.

The convergence half of the row is τ-independent — the combined controller
floors above 1e-4 at every τ from 1e-4 to 1e-1 — so that is the half that is
reported. This is dead claim 5 in chapter 19.

## How it is produced in the study, and a limit that follows

The collapse is applied directly to the target geometry: the ring's
coordinates are squashed along one axis by a factor `c`, with c = 0.02 for the
benchmark.

That is easy in simulation and awkward in reality, which produces one of the
most interesting limits in the project: **with fiducial markers this case is
structurally untestable**. The degeneracy *is* the absence of 2D extent, and an
ArUco marker needs 2D extent to decode. Reproducing the verdict needs c ≤ 0.05;
the smallest collapse that still leaves six decodable markers at this working
distance is c = 0.35 — a factor of seven, and not a resolution limit, because
it persists at 2560 px wide.

So in that regime the guard's trigger condition and the detector's failure
coincide exactly. Real deployments need corner or edge features there, not
markers. That is why the Gazebo cell later uses plain dark dots, which stay
detectable while the geometry collapses (chapter 26).

## Say it like this

> The collapsed target is the case that matters: all six markers visible and
> perfectly detected, and the geometry has gone flat, so one camera motion is
> nearly invisible in the image. Feature counting says six out of six the
> entire time while the partitioned controller floors short of the goal. That
> is the argument for guarding geometry rather than counting features — and
> it's also where I had to retract two of my own explanations, including a
> velocity spike that turned out to be an artifact of the truncation constant.
