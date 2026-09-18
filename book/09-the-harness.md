# 9. The harness: what is actually run

A result is only as good as the setup that produced it, so this chapter states
the setup completely: targets, scenarios, measures, seeds and files.

## The two targets

- **A six-point ring.** Six markers on a circle of radius 0.05 m, on a plane.
  Viewed from about 0.6 m. The main target.
- **A four-point square.** Half-width 0.1 m, viewed from 0.8 m. Used for the
  camera-retreat scenario.

Two targets, not one, because a threshold calibrated on one does not transfer
to the other: the healthy 1st percentile is **3.87e-3** for the ring and
**1.74e-2** for the square, 4.5× apart (chapter 17).

## The four scenarios

| scenario | what happens | why |
|---|---|---|
| camera retreat | the goal is the same pose rotated 180° about the optical axis | the famous coupling failure; positive control |
| feature dropout | 3 of 6 markers vanish for steps 30–90 | conditioning failure |
| two features | 4 of 6 vanish for steps 30–90 | tests the partition's reduced solve |
| collapsed target | all 6 visible, squashed toward a line in 3D (scale 0.02) | information failure that counting cannot see |

Occlusions are a function of the step number, so every controller sees exactly
the same disruption at exactly the same time. That is what makes the
comparison fair, and it is the same principle as putting both Gazebo cells in
one physics world (chapter 23).

## The measures, and why they differ

**Convergence** — final image error below 1e-4 — for retreat and collapse,
where the question is whether the controller arrives at all.

**Velocity spike** — peak commanded speed during steps 30–90, divided by that
controller's own speed at step 29 — for dropout and two-features, where every
controller eventually arrives and the question is how violently.

Dividing by the controller's *own* pre-disruption speed matters: it asks "did
this controller lurch relative to how it was already moving", which is
comparable across laws that run at different speeds. A spike below 1 means the
controller slowed down when information was lost, which is the desired
behaviour.

**Clean cost** — steps to drive the error below 1% of its initial value on an
undisturbed run, relative to classic IBVS. The threshold is part of the
definition: the same comparison gives +2.4% at 50% and +7.4% at 0.01%, so
quoting "+13%" without saying "at 1%" is meaningless.

**Pose error** — distance between the final camera position and the true goal
position — used in the observability chapter, where the image error can be
tiny while the robot is in the wrong place.

## Reproducibility rules

- Thresholds are the healthy 1st percentile over **4000 random poses, seed 0**.
- Every scenario is deterministic given its seed; noise studies state theirs.
- Every number quoted in the README is produced by a script in the repo, and
  the ones that matter are locked by a test so a later change cannot move them
  quietly.

## The files

| file | what it is |
|---|---|
| `ibvs_core.py` | SE(3) helpers, camera, projection, interaction matrix, classic IBVS, σ₆, scenarios |
| `partitioned.py` | the 2001 partitioned law |
| `truncated.py` | adaptive-rank pseudo-inverse and the truncation-only controller |
| `switched.py` | the runtime switch, the six rules, and healthy-pose calibration |
| `tests.py` | 177 regression checks |
| `fd_check.py` | 16 finite-difference checks |
| `compare.py` | regenerates the results table |
| `tau_sweep.py`, `noise_study.py`, `predict.py` | the sensitivity, noise and forward-prediction studies |
| `mj_*.py` | the rendered-camera port with a real ArUco detector |
| `blindspot/` | the shipped library: guard, units, calibration |
| `ros2_ws/src/blindspot_ros/` | the ROS 2 guard node and the RViz simulation |
| `ros2_ws/src/blindspot_arm/` | the Gazebo UR5e cells and the control law they run |

## How to run it

```bash
python3 -m venv .venv && .venv/bin/python -m pip install -r requirements.txt
.venv/bin/python tests.py        # must print 177/177 and 16/16
.venv/bin/python compare.py      # regenerates the results table
```

The dependency list is numpy, scipy and matplotlib. The rendered-camera
scripts additionally need `mujoco` and `opencv-contrib-python`; the ROS parts
need ROS 2; this book's PDF needs `reportlab`. None of those are required to
reproduce the control results, which is deliberate: the core study should be
runnable by anyone in one minute.

## Say it like this

> Two targets, four scenarios, and a measure chosen per failure rather than
> one score for everything — convergence where the question is whether it
> arrives, peak commanded velocity relative to the controller's own
> pre-disruption speed where the question is whether it lurches. Occlusions
> are scripted by step number so every controller meets the identical
> disruption, thresholds come from a fixed seed, and the whole core study runs
> on numpy alone.
