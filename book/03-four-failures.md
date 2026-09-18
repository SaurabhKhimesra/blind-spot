# 3. Four ways to go blind

## In plain words

There isn't one way this goes wrong; there are four, and they are genuinely
different. One is like a puzzle where two answers look equally good and the
solver picks the silly one. One is like losing an eye — a whole direction of
information is gone. One is a fix that becomes a trap when things get thin.
And one is the worst kind: everything *looks* fine, all the marks are visible,
and the information they carry has quietly gone flat.

The last one matters most, because the obvious safety check — "can I still see
all six marks?" — says yes the entire time.

## The four

The project is organised around four failures, chosen because they break
different things. Three of them are the ones a real cell actually meets;
the fourth is a positive control with known ground truth.

| Failure | Scenario in the repo | What breaks |
|---|---|---|
| Camera retreat | 180° rotation about the optical axis | Coupling, not conditioning |
| Feature dropout | 3 of 6 features occluded for 60 steps | A direction genuinely goes blind |
| Two features left | 4 of 6 occluded | The partition's reduced solve becomes brittle |
| Collapsed target | all 6 visible, squashed toward a line | Geometry carries no information, and counting cannot see it |

## 1. Camera retreat — the famous one

Ask classic IBVS to rotate 180° about the optical axis. It flies the camera
**68.19 m** backwards and never converges.

The reason is coupling, not a numerical problem. Nothing is near-singular.
Rotating the image about its centre is, to first order, indistinguishable from
pulling back and coming in again along a spiral, and the least-squares
solution prefers the spiral. Chaumette named this; Corke & Hutchinson fixed
it in 2001; Malis's 2½D scheme fixed it a different way in 1999.

The reason it earns a chapter here is a measurement that shaped the whole
project. **Image error is a useless health signal.** Throughout the retreat,
the alignment between the commanded motion and the error stays at exactly
**−1.0000** — the controller is, by its own arithmetic, doing exactly the
right thing at every step, while the camera flies 68 metres away. Anything
that watches the error will report perfect health during a catastrophic
failure. That is why every candidate health signal in this project is a
property of the *geometry*, never of the error.

## 2. Feature dropout — conditioning

Occlude 3 of 6 features for 60 steps. Classic IBVS spikes to **28.7×** its
pre-occlusion velocity. This is the textbook ill-conditioning case: one
direction of the six loses its supporting information, the pseudo-inverse
amplifies noise along it, and the command jumps.

The textbook fix works: an **adaptive-rank truncated pseudo-inverse**
(chapter 5) simply declines to move along directions whose singular value has
collapsed, and the spike drops to **0.9×** — below the pre-occlusion level,
because the controller is doing *less*.

## 3. Two features left — the partition's own failure

Occlude 4 of 6. Now the counting matters:

- Classic IBVS has 4 equations for 6 unknowns. Underdetermined. The
  pseudo-inverse returns the minimum-norm solution, which is small and gentle.
  Spike: **0.9×**. It is protected by accident.
- The 2001 partition has taken two degrees of freedom out of the inverse, so
  its reduced solve is `L_xy`, a **4×4**. Exactly determined. There is no
  least-squares slack and no minimum-norm protection left, so an
  ill-conditioned 4×4 gets inverted and the command explodes. Spike:
  **288.4×**.

This is the single most important result in the project: **the published fix
for failure 1 is the cause of failure 3.** Adding truncation on top (the
"combined" controller) reduces it to **11.6×** but does not solve it, and 11.6×
is the gap the whole guard exists to close.

## 4. Collapsed target — the one counting cannot see

Take the six-point ring and squash it toward a line in 3D (`P[:,1] *= 0.02`).
All six features are still visible and perfectly detected. Nothing is
occluded. The feature *count* never drops.

But the reduced matrix the partition must invert is
`L_xy = [3.68, 3.66, 0.0124, 0.0037]` — three orders of magnitude between the
strong and weak directions. The partition **forces** all four remaining
degrees of freedom through that matrix, which is exactly what removes
truncation's ability to decline the blind direction.

Result: the partitioned controller floors at **7.3e-3** and the combined one
at **2.0e-3**, neither reaching the 1e-4 convergence bar, while plain
truncation converges to **2.4e-5**.

This case is why the shipped guard is not a feature counter. A rule that
watches how many features are visible reports "6 of 6, all good" while the
controller fails.

Two corrections worth carrying, because both were mine and both were wrong
first:

- The mechanism is **not** "σ = √area → 0 so `vz` blows up". The desired area
  shrinks with the current one, the ratio holds at **1.4980**, and `vz` is
  unchanged at every collapse level. The failure is in the reduced inverse,
  not in the depth feature.
- The collapsed case must be judged on **convergence, not on its velocity
  spike**. At τ=1e-3 the truncation threshold happens to sit at **98.78%** of
  the smallest singular value, so the spike is a 1.2%-margin artifact that
  moves from 5.96 to 0.33 across a τ sweep. The convergence failure is
  τ-independent, so that is the half that counts. (Dead claim 5.)

## Why these four

They separate the two things people conflate. Retreat is a **coupling**
failure with a healthy matrix. Dropout is a **conditioning** failure with a
sick matrix. Two-features is a **structural** failure of one specific
controller. Collapse is an **information** failure that no amount of counting
can detect.

A rule that handles all four cannot be watching just one quantity, which is
why the project spent so long comparing candidate signals instead of picking
the first one that worked.

## Say it like this

> There are four distinct ways an image-based servo goes blind, and they break
> different things. The famous one, camera retreat, is a coupling problem with
> a perfectly healthy matrix — and during it the controller's own error signal
> reads perfect, which is why I never used error as a health measure. The
> published fix for it turns out to cause the worst failure of the other
> three: with two features left it spikes nearly 300 times. That gap is what
> the project set out to close.
