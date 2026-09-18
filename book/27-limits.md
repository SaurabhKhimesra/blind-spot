# 27. Limits, collected

Every limit the project found, in one place. This chapter exists because a
result without its boundary is advertising.

## About the guard itself

**It only wins a race.** The guard watches the polygon area — and that is the
same area the partition drives to its goal. So a degeneracy slow enough for
the control loop to absorb never crosses the threshold. Measured on the
folding part: 85° over 3 s is missed completely and the guarded cell fails
exactly like the unguarded one. It fires at 75° within 1 s, 80° within 1.5 s,
85° within 2 s.

**It cannot tell obliquity from degeneracy.** A healthy planar part viewed at
70° fires the guard at 0.99× while σ₆ reads 27.01× and is right. On the
retreat benchmark from an imperfect start that false positive costs a
2.02–2.88 m retreat in 100 runs out of 100 (chapter 25).

**Calibration cannot buy both.** Widening the healthy tilt range shrinks the
retreat (2.59 m → 1.26 m at 0.9 rad) until, at 1.2 rad, the guard stops firing
and loses the target alongside the partition — and the same widening blinds it
further to a mild collapse (1.19× → 2.46×).

**Beyond it there is still no controller for a collapsed target.** Held
folded, the fallback creeps along an orbit about the hinge line that the image
cannot see (‖v‖ 0.03 → 0.60 over 4 s), and re-enabling the partition
mid-unfold produced a 6.05 command spike that lost the target. The guard
decides *which* law to run; it does not make a law for geometry that carries
no information.

**Three configurations no controller finishes.** Permanently two markers,
three permanently collinear points, and a goal on the three-point danger
cylinder. In all three, classic IBVS, truncation and the switched controller
land on the identical pose error to four decimal places (0.0695 m, 0.1130 m).
The residual is information the features never carried.

## About the thresholds

**They do not transfer between targets.** Healthy 1st percentile: 3.87e-3 for
the ring, 1.74e-2 for the square, 4.5× apart. Calibration is per target.

**One margin is structurally thin.** With a single 6-marker threshold the
dropout case clears by only 1.19×, because a subset's polygon is bounded by
the full set's. Conditioning on the visible count restores 45.9×. That is a
fix for a defect, not a free property.

**Calibrate on a known-good part, never in situ.** In-situ calibration on the
collapsed target gives a threshold 7× lower (7.96e-2 → 1.13e-2), after which
the guard never fires and everything still looks like it is working.

## About the constants

**The shipped τ has a constructible case where it is worse than no
intervention.** On a collapsed target with an initial rotation about the
collapse line, the switched controller at τ=1e-3 stalls at a pose error of
0.297 m while plain classic IBVS converges exactly. τ=1e-5 completes it. It is
a tuning failure, not a structural one, but the default has such a case and τ
is a single global constant chosen from one geometry. Sweep it on yours.

**Clean cost is threshold-dependent.** +13% at a 1% convergence threshold,
+7.4% at 0.01%, +2.4% at 50%. Meaningless quoted without its threshold.

## About the evidence

**The core study uses exact feature positions.** A free-flying virtual camera,
ground-truth features, no detector, no correspondence errors, fixed
dt = 0.033 s, no actuator dynamics, no joint limits, no contact. This is the
single largest open question, and it is why the rendered-camera port and the
Gazebo arm exist.

**Two targets only.** A six-point ring and a four-point square.

**No hardware anywhere.** Rendered cameras and simulated arms are not a real
cell: no lens distortion beyond the model, no rolling shutter, no thermal
drift, no vibration, no real lighting.

**σ₆'s only demonstrated edge over the area guard is measure-zero.** At the
three-point danger cylinder σ₆ = 2.2e-16 while the polygon area is constant to
1.5e-16 relative, so the area guard cannot fire even in principle — but no
sustained trajectory dwells there, and with the goal on the cylinder every
controller floors at about 7e-4 with no winner. A detection difference that
produces no control difference is not a result.

**With fiducial markers the collapsed case is structurally untestable.** The
degeneracy is the absence of 2D extent and an ArUco marker needs 2D extent to
decode: the verdict needs c ≤ 0.05, and the smallest collapse leaving six
decodable markers at this distance is c = 0.35, even at 2560 px wide.

**Correlated noise was tested, and the detector fails first.** Motion blur
driven by the camera's own inter-frame motion: the guard's margin is flat right
up to the cliff (1.62× at 0, 2.5, 5.1 px of blur) and then the detector fails
outright at 12.7 px, which is about a quarter of a marker width. Correlated
degradation does not bias the guard, it removes its input — so a guard failure
and a detection failure stay distinguishable. Still untested: a platform that
holds speed through a degradation window, since this controller is
decelerating by then.

## How to say this in an interview

Volunteer the three that matter before you are asked:

> It's simulation, not hardware. The guard watches a quantity its own
> controller regulates, so a slow collapse is masked — measured, 85° over
> three seconds is missed entirely. And the shipped truncation constant has a
> constructible case where intervening is worse than doing nothing.

Volunteering a limit is the cheapest credibility available, and it shifts the
conversation from "can I catch this person out" to "what would you do next".
