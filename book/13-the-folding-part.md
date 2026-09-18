# 13. The folding part, and the clip

## In plain words

After the tilting panel turned out to be a false alarm, the demo needed a case
where the 2001 controller genuinely cannot cope — not a trick, a real one.

Here it is. The panel is split into two flaps on a hinge, like a book closing.
Mid-run both flaps fold away from the robot, so the six markers **physically
collapse toward a line in space** while every one of them stays in view and
perfectly detected. Nothing is occluded. Nothing is blurry. The part simply
stops being flat.

The 2001 controller judges distance by how big the marker patch looks. The
patch shrinks, so it concludes "I am too far away" and drives at the part. The
guarded arm notices the patch is outside its healthy range and switches to the
plain law, which doesn't use the patch at all, and holds its ground.

## Designing the scenario honestly

The rule I set myself: **gate it in numpy first.** If the unguarded law does
not clearly misbehave and the guarded one clearly hold, there is no clip,
because staging a scenario until the demo looks good is how demos start lying.

The gate used the arm's own control law and measured, for folds from 70° to
85° at speeds from 0.5 s to 3 s:

- the unguarded partition lunges to **2–8 cm** and loses the markers, at every
  fold tested;
- **plain IBVS holds** 20–22 cm at every fold tested, which proves the
  partition is what fails, not the arm or the detector;
- **σ₆ never fires** — minimum 1.29× — because folding *away* from the camera
  adds depth variation and the matrix stays well conditioned;
- the **area guard fires only inside a speed window**.

That last point is the most important thing in this chapter, and it is a
limitation, not a feature. The guard watches the polygon area. The partition's
lunge **restores** that area by getting closer. So a fold slow enough for the
control loop to absorb never crosses the threshold: at 85° over 3 s the area
never drops below 1.16× and the guarded cell fails exactly like the unguarded
one. Fast enough — 75° within 1 s, 80° within 1.5 s, 85° within 2 s — and the
collapse outruns the loop and the guard fires.

**A guard on a quantity its own controller regulates is only as good as that
race.** That sentence is now in the README's limits.

The clip uses 85° in 1 s, comfortably inside the window, and the repo states
the window.

## Building it in Gazebo

Design choices that each fixed a specific problem:

- **Markers are spheres, not discs.** A disc on a flap that has rotated
  85° is seen edge-on and nearly vanishes; a sphere projects as a circle from
  any angle, so the detector keeps working through the whole fold.
- **Sphere centres sit on the flap's mid-plane.** If the markers stand proud
  of the surface, the fold cannot bring their centres toward the hinge line,
  and the collapse in 3D is only partial.
- **The marker ring is rotated 11°.** At 0° two pairs of markers share the
  same column, so they merge into one blob as the pattern flattens and the
  detector drops to five features. 11° keeps every column distinct.
- **A light backdrop sits 11 cm behind the hinge**, so a dark marker never
  lands on a dark background as the flaps swing back.
- **The flaps fold away from the arm**, so the arm can never touch them and
  the physics stays clean.
- **Both cells run the same protective stop**: no commanded motion may bring
  the camera within 12 cm of the hinge plane, checked on the *next* pose.
  Identical rule on both sides, so the comparison stays fair. This is also
  what keeps the unguarded failure realistic rather than a simulated crash.

## What one run measured

Both cells are identical until the fold. Then:

| | partition always on | with the guard |
|---|---|---|
| guard decision | not used | **fires 0.96 s into the fold**, margin 0.90× |
| standoff | 26.3 cm → **protective stop at 16.2 cm**, 1.46 s in | **holds 22–24 cm** |
| markers detected | 6/6 throughout | 6/6 throughout |

And the same act in numpy with the same law: the partition reaches 6.8 cm and
loses the markers for 38 steps; the guard fires 0.9 s in and holds 18 cm;
plain IBVS holds 22.5 cm. Locked as `tests.py` section 22.

## What the act deliberately does not show

Two things were measured and are **not** in the clip, both in the README's
limits:

- **The fallback creeps.** Held folded for several seconds, the plain law's
  commanded speed grew from 0.03 to 0.60 over 4 s and the marker
  correspondence started flipping. A fully folded part leaves an orbit about
  the hinge line that the image cannot see — the same weak-direction story as
  chapter 12 — and the controller drifts along it.
- **Re-engaging the partition mid-unfold is violent.** When the flaps opened
  again, the guard re-enabled the partition while the part was still folded to
  62°, the first command after that was **6.05**, and the target was lost
  within a second.

So the act ends 2.6 s after the fold, and the clip claims only what the act
shows. The guard decides *which* law to run; it does not make a law for
geometry that is genuinely unobservable.

## How the clip was made

1. `run_fold_demo.sh` with `RECORD=1` runs the act and saves **every frame**
   from two film cameras (60 Hz) and both wrist cameras (20 Hz), each named by
   its simulation timestamp, plus a 10 Hz CSV of everything the controller
   knew.
2. Gazebo is slowed to about a quarter of real time so the 1080p cameras never
   drop a frame. Because every node runs on simulation time, this changes
   nothing about the control loop.
3. `arm_clip.py` cuts the clip from those frames and that log: hook, title,
   approach, the fold in slow motion with live wrist views and the guard
   gauge, then the logged distance curves, then two end cards.

The rules I held myself to, which are written at the top of `arm_clip.py`:

- every frame of robot motion is a frame Gazebo rendered during that run;
- every number on screen is read from that run's log;
- the edit changes only timing, crops and overlays — no re-staging, no
  re-enactment, no numbers typed in by hand;
- the end card states the limits rather than hiding them.

A produced clip is fine. A fabricated one would make every number in the repo
worthless, which is the actual asset.

## Say it like this

> The tilting panel was a false alarm, so I built the case the 2001 law
> genuinely cannot survive: a part that folds, so the markers collapse toward
> a line in 3D while all six stay perfectly visible. The controller reads the
> shrinking pattern as distance and drives at the part; it trips the cell's
> protective stop at 16 cm. The guarded arm switches a second into the fold
> and holds 22. I gated the whole scenario in numpy before filming it, and the
> clip's end card says the part the demo can't show: the guard only catches
> folds faster than the control loop can absorb.
