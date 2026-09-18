# 2. Where things are: frames and transforms

Everything later is about a camera and a part, so we need a precise way to say
where something is. This chapter builds that from scratch. If you know
rotation matrices already, skim to the last section, which fixes the
conventions this project uses.

## A position is three numbers, relative to something

"The marker is at (0.05, 0, 0)" means nothing on its own. Relative to what?

A **frame** is a fixed origin plus three directions (x, y, z) that everything
is measured against. A position is three numbers **in a stated frame**. Change
the frame and the numbers change, while the marker has not moved at all.

This project uses three frames constantly:

- the **object frame**: attached to the part. The six markers have fixed
  coordinates here, and they never change.
- the **camera frame**: attached to the camera. `z` points out of the lens,
  `x` right, `y` down. This is the computer-vision convention and it is what
  the code means by "the camera's view".
- the **world frame**: the robot cell. Only the arm chapters need it.

## Rotation as a matrix

Turning something in 3D can be written as multiplying by a 3×3 matrix `R`.
Rotating by angle θ about the z axis:

```
Rz(θ) = [ cos θ   −sin θ   0 ]
        [ sin θ    cos θ   0 ]
        [   0        0     1 ]
```

Check it on a simple case. Take the point (1, 0, 0) and θ = 90°. Then
cos θ = 0, sin θ = 1, and the product is (0, 1, 0): the point on the x axis
has swung onto the y axis. That is what a 90° turn about z should do.

Rotation matrices have two properties worth remembering, because the code
relies on both:

- **The inverse is the transpose.** `R⁻¹ = Rᵀ`. Undoing a rotation costs
  nothing.
- **They compose by multiplication, right to left.** `R₁ R₂` means "do R₂
  first, then R₁". Order matters: turning right then up is not the same as
  turning up then right.

## Position and rotation together: the 4×4

A rigid body has both an orientation and a position, so you keep both in one
4×4 matrix:

```
T = [ R   t ]        R = 3×3 rotation
    [ 0   1 ]        t = 3×1 position
```

The trick that makes this worth doing: to move a point `p` from one frame to
another you write it as (x, y, z, 1) and multiply. The bottom row of ones is
what lets a single matrix multiplication do "rotate **and** shift", which a
3×3 cannot.

```
p_new = T · [p; 1]        →      p_new = R·p + t
```

Two of these compose the same way rotations do: `T_ac = T_ab · T_bc`. This is
the whole of `make_pose`, `transform_points` and the pose bookkeeping in
`ibvs_core.py`.

The notation used throughout the repo is `cTo`: the pose of the **o**bject
expressed in the **c**amera frame. Read it right to left — "object, into
camera". So `transform_points(cTo, P)` takes marker coordinates written in the
object frame and returns where they sit in the camera's frame.

## Velocity: the twist

A moving rigid body has a linear velocity and an angular velocity. Stack them
into six numbers and call it a **twist**:

```
v = (vx, vy, vz, ωx, ωy, ωz)
```

The first three are metres per second, the last three radians per second.
Every control law in this project outputs exactly this: six numbers, the
camera's commanded twist.

If a point sits at `p` in the camera frame and the *camera* moves with twist
`v`, the point's coordinates in the camera frame change as

```
ṗ = −v_linear − ω × p
```

Both minus signs are there because the camera moves, not the point: if the
camera slides right, everything in its frame slides left. This equation is the
seed of the entire project — chapter 7 differentiates the camera projection
using exactly this, and out falls the interaction matrix.

## From twist to pose: the matrix exponential

A twist says how fast; a pose says where. To apply a twist for a short time
`dt` you need the pose update. For rotations you cannot just add — adding
angles works in 2D, not in 3D — so the correct operation is the **matrix
exponential**, written `se3_exp`:

```
cTo ← se3_exp(−v · dt) · cTo
```

Think of it as "follow this constant velocity for dt seconds, exactly, along
the curved path rotations actually take". Its inverse, `se3_log`, turns a pose
difference back into the twist that would produce it. Both live in
`ibvs_core.py` and both are verified by finite difference (chapter 18),
because a sign slip here produces a controller that behaves oddly in a way you
will wrongly blame on the control law.

The minus sign in front of `v` is the same bookkeeping as before: the twist
describes the camera's motion, and `cTo` describes where the object is *as
seen from* the camera.

## Say it like this

> Positions are three numbers in a named frame; poses are a rotation and a
> translation kept in one 4×4 so a single multiply moves a point between
> frames. Velocities are six numbers, a twist. Applying a twist to a pose uses
> the matrix exponential rather than addition, because rotations do not add.
