# 24. Kinematics and perception on the arm

Two things stand between the control law and the robot: turning a camera twist
into joint motion, and turning an image into marker positions. This chapter is
both, and both were written by hand so that they could be checked.

## From camera twist to joint commands

The control law outputs a twist for the **camera**. The robot accepts joint
positions. Three steps bridge them.

**1. Forward kinematics.** Where is the camera, given the joint angles? Walk
the chain of links from the base to the camera frame, multiplying each joint's
4×4 transform (chapter 2). That is `kin.py`: it parses the URDF with
`urdf_parser_py` and composes the transforms in numpy.

**2. The geometric Jacobian.** How does the camera move when each joint moves?
For a revolute joint with axis `z_i` (in base coordinates) whose origin is at
`p_i`, and with the camera at `p_e`:

```
linear part  of column i = z_i × (p_e − p_i)
angular part of column i = z_i
```

Stack the six columns and you have `J`, a 6×6 matrix with `twist = J · q̇`.
The intuition for the cross product: rotating about an axis moves a point at
distance `r` from it with speed `ω × r`.

**3. Invert it.** `q̇ = J⁺ · twist`, then clip each joint velocity to
±1.5 rad/s, integrate one step (`q_next = q + q̇·dt`), and publish `q_next` as
a single-point trajectory.

Note that this is the *same* mathematics as the visual servo itself — a matrix
mapping motion to observation, inverted to get the command — one level down.
And it has the same failure mode: near a kinematic singularity `J` becomes
ill-conditioned and the joint speeds explode. The velocity clip is the crude
protection; a robot cell would use a damped least-squares inverse.

## Why hand-written kinematics is defensible here

No KDL, no MoveIt, about 90 lines of numpy. The reason it is trustworthy is
not that it is short — it is that it is **checked against something
independent**: `verify_against_tf()` compares this forward kinematics to what
`robot_state_publisher` reports over TF, which is computed by different code
from the same URDF.

**Agreement: 1e-6 m.** A wrong joint axis, a missed fixed joint or a
transposed rotation shows up immediately at that tolerance. This is chapter 18's
principle applied to robot geometry: do not reason about whether your chain is
right, measure it against an independent implementation.

## Perception: finding six dots

The markers are plain dark circles on a light panel, and the detector is
deliberately simple (`detect.py`):

1. convert to grayscale;
2. threshold: anything darker than 90 is marker-ish;
3. connected components — group touching dark pixels into blobs;
4. keep blobs whose area is between 25 and 4000 px, which rejects specks and
   large dark regions;
5. from those, pick **six** and return their centroids.

Step 5 is where the interesting bug lived. The obvious rule is "take the six
largest", and it fails: at close standoff the robot's own dark wrist enters
the frame and is *bigger* than any marker, so it displaces a real one and the
controller servos on a lie.

The fix is to pick the six blobs most **similar in area** to each other,
because the six real markers are identical circles at similar depths. It is a
one-line change and it is the difference between a demo that works and one
that fails only at close range, which is the worst time to fail.

## Why plain dots and not ArUco

ArUco markers carry ids, which solves correspondence for free. They also need
2D extent to decode — and the whole point of the folding scenario is that the
2D extent collapses (chapter 13's structural limit). At the fold angles that
matter, ArUco stops decoding before the geometry is interesting.

Dark dots stay detectable all the way through the collapse. And in the folding
world they are **spheres**, not discs, so that a marker on a flap rotated 85°
still projects as a circle instead of vanishing edge-on.

The price is correspondence, which has to be solved explicitly.

## Correspondence, explicitly

The detector returns six centroids in arbitrary order. To subtract `s*` from
`s` you need to know which is which.

**Step 1: order by angle.** Compute the centroid of the six points and sort
them by the angle around it. For points on a ring this fixes the cyclic order
regardless of how the detector found them.

**Step 2: fix where the order starts.** Angular ordering does not say which
dot is "first", so a rotated target can pair every dot with the wrong partner.
Measured consequence: one cell converged to an error of 0.090 while its
identical twin diverged to 0.530, running the same code.

The fix is to try all six cyclic shifts of the desired features and keep the
one with the smallest residual:

```python
best = min((np.linalg.norm(s - np.roll(s_star, k, axis=0)), k) for k in range(6))
```

Six comparisons per step, and the ambiguity is gone.

## Intrinsics, and a stall that looked like a control bug

The node needs focal length and principal point to convert pixels to
normalised coordinates. The natural source is `camera_info`, and on this setup
it never arrived on the ROS side. Worse, waiting for it stalled the servo loop
silently: the HUD read 0/6 features and neither arm moved, which looks exactly
like a controller problem.

The fix is to take the intrinsics from the sensor definition in the URDF,
where they are already declared, with the `(W−1)/2` principal point convention
from chapter 20 — and to write a comment saying why, so that the next person
does not "fix" it back.

## The control node, end to end

Per cell, ten times a second:

1. latest wrist camera frame;
2. detect the six dots;
3. order by angle, convert to normalised coordinates;
4. ask the guard for a reading (calibrated at start-up on the known-good
   hexagon);
5. pick the control law — left cell always partition, right cell obeys the
   guard;
6. compute the twist with the **same law the numpy study uses**, imported from
   `law.py`, which `tests.py` also imports, so the tests check the law the arm
   actually runs;
7. Jacobian, clip, integrate, publish;
8. apply the shared **protective stop** if the next pose would come within
   12 cm of the part;
9. log everything to CSV at 10 Hz.

Step 6 is the part worth pointing at in an interview. There is exactly one
implementation of the control law in the repository, and both the regression
tests and the robot import it. A study whose conclusions are checked against a
*copy* of the deployed code is checking the wrong thing.

## Say it like this

> The control law produces a camera twist, so I map it through the arm's
> geometric Jacobian — cross product of joint axis with the lever arm, written
> by hand in numpy from the URDF and verified against TF to a micrometre —
> clip joint speeds, integrate a step and publish a trajectory point.
> Perception is deliberately simple: threshold, connected components, and pick
> the six blobs most similar in area, because picking the six largest lets the
> robot's own wrist masquerade as a marker at close range. Correspondence is
> six cyclic shifts, chosen by residual.
