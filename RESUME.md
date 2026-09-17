# Where this stopped, and what to do next

Written 2026-09-16 so the arm work can be picked up cold. Updated 2026-09-17.

## State

Everything is committed and pushed to
https://github.com/SaurabhKhimesra/blind-spot

- `tests.py` 177/177, `fd_check.py` 16/16, `test_blindspot.py` 34/34
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

The spike is on the GUARDED cell and coincides in time with the guard
switching laws. The unguarded cell stays smooth. Coincidence in time is all
that was observed; see the correction below.

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
(the 16th differed by 5%). On a tilting target with little error left, the two
laws compute nearly the same twist. The single arm spike is unreproduced; the likelier culprit is
perception, since at 66 degrees the dots are thin ellipses and blob centroids
get noisy. Locked in tests.py section 20.

## The finding this produced, which is worth more than the video

**The area guard has a false-positive mode.** The polygon-area signal cannot
distinguish

- a target that is geometrically degenerate (dangerous, partition must go)
- a healthy target merely viewed at a steep angle (fine, partition should stay)

Both shrink the projected area. The numpy study could not have
found this: a free-flying camera never had a reason to view a healthy target
that obliquely.

Written up 2026-09-17 as a revision of dead claim 4, locked in tests.py
section 20. It does more than add a limit: it partly revives the claim that
the spectrum matters, which the four-case suite had wrongly killed.

## Found 2026-09-17: the retreat row hides a trade, not a win

Checking whether "area is more noise-robust" was actually measured turned up
a bigger problem. The retreat benchmark starts from an EXACT rotation about
the optical axis, where the partition's xy command is exactly zero. From a
start just 2 cm or 3 deg off, all of that command lands in the two weakest
directions of L_xy (about 130x below the strongest), which are an orbit about
the target, and the view swings nearly edge-on. 100 random starts at 180 deg:

- partition always on / sigma_6: never switch, 0.80 m, but 3 of 100 go fully
  edge-on and lose the target (peak tilt median 84, max 89 deg)
- area guard: fires 100 of 100 on the foreshortening, never loses the target,
  but its fallback retreats to 2.02-2.88 m

Widening the area calibration's tilt range only moves along the trade (at
1.2 rad it stops firing and loses the target too). 0.5 px of noise triggers
the same orbit from the exact start. The area statistic IS about 3x steadier
under noise; the closed-loop switch is not. Written into README dead claim 4
and the Results notes, locked in tests.py section 21.

**Open decisions for the user, not taken:**
1. The README headline ("Guard the features ... not the matrix it inverts")
   is now contested by the repo's own evidence. Neither signal dominates.
2. The `blindspot` package ships only the area guard. Whether to offer
   sigma_6, or both, is a design change.
3. The video path below.

## Resolved 2026-09-18: the clip exists, path B

Built as the folding part: each panel is two flaps on a hinge, both fold 85
deg in 1 s away from the arm, and the markers collapse toward the hinge line
in 3D with all six still visible. This is the case the partition cannot
survive - it reads the shrinking area as distance and drives at the part.

- Gazebo, one run: unguarded trips the shared protective stop at 16.2 cm,
  1.46 s into the fold; guarded fires at 0.96 s and holds 22-24 cm.
- numpy, same law imported from the demo package: 6.8 cm and markers lost vs
  18 cm held; plain IBVS holds 22.5 cm; sigma_6 never fires (1.29x).
- The guard only wins a race: 75 deg within 1 s, 80 within 1.5, 85 within 2.
  Slower folds are masked because the lunge restores the area.
- Not shown in the act, and in the README limits: the fallback creeps while
  the part is held folded, and re-enabling the partition mid-unfold spiked to
  6.05 and lost the target.

Files: worlds/panel_fold.sdf, blindspot_arm/{fold_node,law,detect,record_node}.py,
launch/fold.launch.py, run_fold_demo.sh, arm_clip.py, docs/folding_part.mp4,
tests.py section 22. Recording runs live outside the repo in ~/blindspot_clip.

Still open, and mine to decide, not Claude's: the README headline, and
whether the package should offer sigma_6 as well as the area guard.

## The two paths that were considered

**A - two-feature occlusion.** Occlude 4 of 6 markers. The numpy study
measured partitioned 288x against guard 0.5x, so the effect is huge and real
and will look violent on an arm. Cost: feature counting can also see this, so
it loses the "only this idea catches it" angle.

**B - collapse the geometry in 3D.** Mount the dots on two hinged flaps that
fold toward each other so the markers approach a line in space. Reproduces the
actual degeneracy and keeps the unique angle. More build work, and the guard
must be shown to fire AND help before promising the shot.

Recommendation was B, with A as fallback. Still stands after 2026-09-17, with
one new constraint: the flaps must fold without tilting the view, or the
area guard fires on obliquity instead of on the collapse.

## Rules that held throughout

- The repo stays honest; the video may be produced well but never fabricated.
- Report bad numbers straight. This file exists because of that.
- `pkill -f <pattern>` matches the running shell's OWN command line. Use a
  bracketed pattern (`[d]uel_node`) or it kills the session.
- Gazebo's process is `gz-sim-server`, not `gz sim`. Stale servers stack up
  and silently corrupt measurements across runs.
