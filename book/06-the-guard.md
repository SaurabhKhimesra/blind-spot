# 6. The guard

## In plain words

The 2001 controller leans on one measurement: the size of the patch the
markers cover. The guard's job is to notice when that measurement has stopped
being trustworthy.

How do you know what "trustworthy" looks like? You measure it in advance on a
part you know is fine. Put the camera in thousands of reasonable positions
around the good part, record the patch size each time, and take a value near
the bottom of that range as the alarm line. Later, in operation, if the patch
size falls under that line, the controller's shortcut is no longer safe, and
the guard says so.

That's the whole idea. One number, one comparison, per step.

## What it computes

```python
sigma = sqrt(polygon area of the visible features)   # shoelace formula
margin = sigma / threshold_for(number_of_features)
decision = margin > 1
```

The shipped API (`blindspot/guard.py`) returns all of it, not just the
boolean, because a decision you cannot inspect is a decision you cannot debug:

```python
GuardReading(signal, threshold, margin, n_features, decision)
```

## Calibration, and why it is a procedure not a constant

`blindspot/calibrate.py` draws random healthy poses around the goal:

- translation: x, y uniform in ±lateral, z uniform in the working range
- rotation: a random roll, then small random tilts (±0.3 rad by default)
- any pose that puts a feature behind the camera is rejected and redrawn

It records σ at each pose and takes the **1st percentile** as the threshold.
Default 4000 poses, seed fixed, so the number is reproducible.

Three properties of that procedure matter:

**It is per target.** The healthy 1st percentile is 3.87e-3 for the six-point
ring and 1.74e-2 for the four-point square — **4.5× apart**, because the
statistic carries the target's scale and the working distance. There is no
global constant, which is why `FeatureGuard(None)` raises an error with the
calibrate command in the message rather than shipping a default.

**It is per visible count.** Singular-value interlacing means a subset's
statistic is bounded above by the full set's. Calibrate at six features and
operate at three and the dropout case clears the threshold by only **1.19×** —
structurally thin, and it looks like a healthy margin until you know why it is
there. Conditioning the calibration on the number of features visible restores
it to **45.9×**. Below three points there is no polygon at all, so the
threshold at N=2 is exactly 0 and the guard always fires.

**It must be done on a known-good target.** This is the deployment trap with
no error message. Calibrating on whatever the camera happens to see makes the
degeneracy the norm: the 1st percentile of a *degenerate* target's own
distribution sits below its everyday value, so the guard never fires.
Measured on the collapsed target: known-good threshold **7.96e-2**, in-situ
threshold **1.13e-2**, a factor of seven. With the good threshold the guard
fires and the run converges (2.7e-5). With the in-situ one the partition stays
on for all 600 steps and the run floors at 2.7e-3 — silently disabled, and
everything still looks like it is working.

## Why the area and not the matrix

The design claim is: **guard the features your controller is built on, not the
matrix it inverts.** The partition's substitute for depth is the area, so the
area is what has to be healthy for the partition to be safe.

Supporting arguments, both measured:

- **It is cheap.** A shoelace area over the points already in hand. No SVD.
- **It is steady.** An area is an aggregate over every visible point, so
  independent per-point detector noise largely cancels inside it. 2 px of
  per-point noise moves it by **0.33%** of its threshold on the square and
  **0.76%** on the ring; the same noise moves σ₆ by **1.03%** and **2.35%**,
  about 3× as much.

And the honest counterweight, which is in the README and in chapter 12 here:
in closed loop σ₆ produced no noise-induced switch at all, while the area
guard's noise-induced switch on the retreat case is a real dip in *area* that
is not a dip in target health. The steadier statistic did not translate into
the steadier decision.

## Chatter and hysteresis

A guard that flickers is worse than no guard. Measured across all four cases
at 0, 0.5 and 2 px of feature noise: **no spurious flips at 0 and 0.5 px, and
one isolated single-step blip at 2 px** on the retreat case only.

Hysteresis (requiring the decision to persist before acting) was implemented,
measured and **rejected**. It suppresses that one blip, but it delays *both*
edges, and the edge that matters is dropping the partition the instant the
degeneracy appears. On the two-feature case the spike goes from **0.5×** with
no hold-off to **3.0×** (hold 2), **3.7×** (hold 3) and **5.7×** (hold 5). The
parameter still exists in `run_switched` so the result can be re-checked
rather than taken on trust.

## What the guard does not do

It decides *which law to run*. It does not invent a law for geometry that
carries no information. The README has three configurations where no
controller can finish — two features permanently, three collinear points, the
three-point danger cylinder — and in all three, classic IBVS, truncation and
the guard-switched controller land on the **identical pose error to four
decimal places** (0.0695 m, 0.1130 m). The residual is not something a better
controller recovers. It is information the features never carried.

Stating that is part of the product. A guard that implies it can rescue any
situation is a guard that will be trusted in situations it cannot.

## Say it like this

> The guard watches one number: the area of the polygon the markers make,
> which is exactly the quantity the 2001 controller uses as a stand-in for
> distance. The alarm line comes from calibration on a known-good part — a
> few thousand healthy viewpoints, first percentile, per target and per number
> of markers visible. If you calibrate it on the part you're currently
> servoing to, you silently disable it, and I measured that: the threshold
> comes out seven times lower and the guard never fires again.
