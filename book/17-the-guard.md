# 17. The guard that shipped

## In plain words

The 2001 controller leans on one measurement: the size of the patch the
markers cover. The guard's job is to notice when that measurement has stopped
being trustworthy.

How do you know what trustworthy looks like? Measure it in advance on a part
you know is good. Put the camera in a few thousand reasonable positions around
it, record the patch size each time, and take a value near the bottom of that
range as the alarm line. In operation, if the patch falls below the line, the
controller's shortcut is no longer safe.

One number, one comparison, per step.

## What it computes

```python
sigma  = sqrt(polygon area of the visible markers)      # shoelace formula
margin = sigma / threshold_for(number_of_markers_seen)
decision = margin > 1
```

The API returns all of it, not just the boolean, because a decision you cannot
inspect is a decision you cannot debug:

```python
GuardReading(signal, threshold, margin, n_features, decision)
```

The shoelace formula computes a polygon's area from its corners in order — and
"in order" is load-bearing. A self-crossing order gives a small area and fires
the guard for a reason that has nothing to do with the part. Chapter 22 is how
the ROS node deals with that when a real detector hands over points in
arbitrary order.

## Calibration, as a procedure

`blindspot/calibrate.py` draws random healthy poses around the goal:

- translation: x, y uniform within a lateral range, z uniform within the
  working distance range;
- rotation: a random roll about the optical axis, then small random tilts
  (±0.3 rad by default);
- any pose putting a marker behind the camera is rejected and redrawn.

It records σ at every pose and takes the **1st percentile**: 4000 poses,
fixed seed, reproducible. That percentile is a false-alarm budget — at 1%, the
guard gives up the partition on one per cent of perfectly healthy views, which
costs almost nothing because the fallback is itself a competent controller.

Three properties of the procedure matter more than the number it produces.

**Per target.** Healthy 1st percentile: **3.87e-3** for the six-point ring,
**1.74e-2** for the four-point square — 4.5× apart, because the statistic
carries target scale and working distance. No global constant exists, which is
why `FeatureGuard(None)` raises an error containing the calibrate command
rather than shipping a default.

**Per visible count.** A subset's polygon is always smaller than the full
set's, so comparing a 3-marker measurement against a 6-marker threshold is
unfair by construction. Calibrated at six and operated at three, the dropout
case clears its threshold by only **1.19×**; conditioned on the count, the same
instant reads **45.9×**. Below three markers there is no polygon at all, so the
threshold at two is exactly 0 and the guard always fires.

**On a known-good part, never in situ.** This is the deployment trap with no
error message. Calibrating on whatever the camera happens to be looking at
makes the degeneracy the norm. Measured on the collapsed target: known-good
threshold **7.96e-2**, in-situ threshold **1.13e-2**, a factor of seven. With
the good threshold the guard fires and the run converges (2.7e-5); with the
in-situ one the partition stays on for all 600 steps and the run floors at
2.7e-3. The guard is silently disabled and everything still looks like it is
working.

## Why this signal rather than the matrix

The design claim: **guard the features your controller is built on, not the
matrix it inverts.** The partition's substitute for depth is the area, so the
area is what must be healthy for the partition to be safe.

Supporting measurements:

- **Cheap**: a shoelace area over points already in hand, no SVD.
- **Steady**: an area is an aggregate over every visible point, so independent
  per-point noise largely cancels. 2 px of per-point noise moves it by
  **0.33%** of its threshold on the square and **0.76%** on the ring, against
  **1.03%** and **2.35%** for σ₆ — about 3× steadier.

And the honest counterweight, measured later and now in the README: in closed
loop σ₆ produced **no** noise-induced switch at all, while the area guard's
noise-induced switch on the retreat case is a real dip in area that is not a
dip in target health (chapter 25). The steadier statistic did not produce the
steadier decision.

## Chatter, and why there is no hysteresis

A guard that flickers is worse than no guard. Measured across all four cases
at 0, 0.5 and 2 px of feature noise: **no spurious flips at 0 and 0.5 px, and
one isolated single-step blip at 2 px**, on the retreat case only.

Hysteresis — requiring the decision to persist for a few steps before acting —
was implemented, measured, and **rejected**. It suppresses that blip, but it
delays *both* edges, and the edge that matters is dropping the partition the
instant the degeneracy appears. On the two-feature case:

| hold-off | spike |
|---|---|
| none | **0.5×** |
| 2 steps | 3.0× |
| 3 steps | 3.7× |
| 5 steps | 5.7× |

The cost of a late drop dwarfs the cost of one spurious blip. The parameter
still exists in `run_switched` so the result can be re-checked rather than
believed.

## What the guard does not do

It decides *which law to run*. It does not invent a law for geometry that
carries no information.

Three configurations in the study cannot be finished by any controller:
permanently two markers, three permanently collinear points, and a goal on the
three-point danger cylinder. In all three, classic IBVS, truncation and the
guard-switched controller land on the **identical** final pose error to four
decimal places (0.0695 m, 0.1130 m). The residual is not something a better
controller recovers; it is information the features never carried, and the
missing capability is perception or planning.

Saying that is part of the product. A guard that implies it can rescue
anything will be trusted where it cannot.

## Say it like this

> The guard watches one number: the area of the polygon the markers make,
> which is exactly the quantity the 2001 law uses as a stand-in for distance.
> The alarm line comes from calibration on a known-good part — thousands of
> healthy viewpoints, first percentile, per target and per visible count,
> because a subset's polygon is structurally smaller. Calibrate it in situ and
> you silently switch it off: I measured the threshold coming out seven times
> lower, after which it never fires again.
