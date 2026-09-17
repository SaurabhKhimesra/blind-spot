# Where this stopped, and what to do next

Written 2026-09-16 so the arm work can be picked up cold.

## State

Everything is committed and pushed to
https://github.com/SaurabhKhimesra/blind-spot

- `tests.py` 143/143, `fd_check.py` 16/16, `test_blindspot.py` 34/34
- ROS 2 **Lyrical** at `/opt/ros/lyrical` (apt). The RoboStack conda env in
  `~/micromamba/envs/rosguard` is now redundant and can be deleted (~6.4 GB).
- The Gazebo arm demo runs with two scripts, no env setup:

```bash
~/blind-spot/run_arm_demo.sh     # Gazebo + two UR5e cells + servo loop
~/blind-spot/view_hud.sh         # second terminal: side-by-side HUD
```

Everything uses `ROS_DOMAIN_ID=7`. Both scripts clear `LD_LIBRARY_PATH`
because a shell that has sourced the conda ROS env breaks system binaries.

## The open question

The arm demo was built to show: panel tilts toward edge-on, guard fires, the
unguarded cell lurches. **The measurement says the opposite.**

```
t=21.6   LEFT  (guard OFF)  |v| 0.259   peak 0.526
t=21.6   RIGHT (guard ON)   |v| 1.231   peak 2.561
```

The spike is on the GUARDED cell, exactly at the moment the guard switches
laws. The unguarded cell stays smooth.

**CORRECTED 2026-09-17.** The paragraph that stood here claimed the spike
was *caused by the switch* - "a pure discontinuity between two control laws".
That was attributed too quickly and did not survive measurement.

What holds: tilting a flat panel is not a 3D collapse. The six dots stay a
perfect hexagon in space and only the projection foreshortens, so the area
guard fires while the interaction matrix is actually HEALTHIER (sigma_6 rises
about 15x with tilt).

What does not hold: that switching laws caused the spike. Reproduced in numpy
across 16 tilt onset/speed combinations, the area guard switched every time
yet peak commanded velocity was identical under all three rules in 15 of 16
(the 16th differed by 5%). On a healthy target the two laws compute nearly the
same twist. The single arm spike is unreproduced; the likelier culprit is
perception, since at 66 degrees the dots are thin ellipses and blob centroids
get noisy. Locked in tests.py section 20.

## The finding this produced, which is worth more than the video

**The area guard has a false-positive mode.** The polygon-area signal cannot
distinguish

- a target that is geometrically degenerate (dangerous, partition must go)
- a healthy target merely viewed at a steep angle (fine, partition should stay)

Both collapse the projected area identically. The numpy study could not have
found this: a free-flying camera never had a reason to view a healthy target
that obliquely. This belongs in the README as a dead claim or a limits entry,
with the numbers above.

Written up 2026-09-17 as a revision of dead claim 4, locked in tests.py
section 20. It does more than add a limit: it partly revives the claim that
the spectrum matters, which the four-case suite had wrongly killed.

## Two paths to a dramatic, honest clip

**A - two-feature occlusion.** Occlude 4 of 6 markers. The numpy study
measured partitioned 288x against guard 0.5x, so the effect is huge and real
and will look violent on an arm. Cost: feature counting can also see this, so
it loses the "only this idea catches it" angle.

**B - collapse the geometry in 3D.** Mount the dots on two hinged flaps that
fold toward each other so the markers approach a line in space. Reproduces the
actual degeneracy and keeps the unique angle. More build work, and the guard
must be shown to fire AND help before promising the shot.

Recommendation was B, with A as fallback.

## Rules that held throughout

- The repo stays honest; the video may be produced well but never fabricated.
- Report bad numbers straight. This file exists because of that.
- `pkill -f <pattern>` matches the running shell's OWN command line. Use a
  bracketed pattern (`[d]uel_node`) or it kills the session.
- Gazebo's process is `gz-sim-server`, not `gz sim`. Stale servers stack up
  and silently corrupt measurements across runs.
