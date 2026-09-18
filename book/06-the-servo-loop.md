# 6. The servo loop

## In plain words

Guiding your hand to a doorknob: you don't measure centimetres, you look, see
the knob is too far left, move right, look again. Repeat until it's where it
should be.

That is the whole loop. The robot's version replaces "where the knob is in
your view" with "where six marker dots are in the camera image", and replaces
"move right" with six numbers describing how the camera should move.

## Two ways to build it

**Position-based (PBVS).** Use the image to estimate the full 3D pose of the
part, then drive the *pose* error to zero.

- Good: you reason in metres and degrees, which is intuitive, and the path
  through space is a straight line.
- Bad: you need a model of the target and a well-calibrated camera, and while
  the pose error looks fine the markers can drift out of the picture, at which
  point you have nothing.

**Image-based (IBVS).** Define the error *in the image* and drive that to
zero. No pose estimate anywhere.

- Good: features stay in the frame, because keeping them where they should be
  in the picture *is* the objective. Robust to calibration error.
- Bad: the relationship between camera motion and image motion is nonlinear
  and depends on depth, and the path through space can be strange.

This project is entirely IBVS, because the interesting failure modes live in
that nonlinear relationship.

## The pieces

**Features `s`.** What the camera measures. Here: six marker centres, each a
pair of normalised coordinates (chapter 3), so `s` is 12 numbers.

**Desired features `s*`.** Where those measurements should be when the robot
is in the right place. Recorded once by putting the robot at the goal and
looking, or computed from the known target geometry and the desired standoff.

**Error `e = s − s*`.** 12 numbers, in image units, not metres.

**Command `v`.** Six numbers: the camera's twist (chapter 2).

The loop, per step:

```
1. take a picture
2. find the markers            -> s
3. e = s - s*
4. build the interaction matrix L from s and depth
5. v = -lambda * pinv(L) @ e
6. send v to the robot, wait dt, repeat
```

Steps 4 and 5 are the whole subject of the next two chapters. Step 2 is
perception, which is assumed perfect for most of the study and then made real
twice: with an ArUco detector on rendered images (chapter 20) and a blob
detector on a simulated arm (chapter 24).

## One subtlety: correspondence

Step 3 subtracts two lists. That only means something if entry *k* of `s` and
entry *k* of `s*` are the **same physical marker**. Getting that pairing right
is called **correspondence**, and with six identical dots it is not free.

Ordering the detected dots by angle around their centre fixes their order but
not where the order *starts*, so a rotated target can pair every dot with the
wrong partner. In the Gazebo cell that produced a beautiful bug: one arm
converged to an error of 0.090 while its identical twin diverged to 0.530,
running the same code. The fix is to try all six cyclic shifts and keep the
one with the smallest residual (chapter 24).

Markers with ids — ArUco, AprilTag — solve this by construction, which is why
the shipped ROS node orders detections by id and only falls back to angular
ordering when ids are absent.

## What "converged" means

Two different measures are used in this book, and they are not
interchangeable:

- **Convergence**: the final image error falls below 1e-4. Used for failures
  where the question is "does it get there at all".
- **Velocity spike**: the peak commanded speed during the disruption, divided
  by that same controller's speed just before it. Used for failures where the
  controller does arrive but lurches on the way.

A single score would hide exactly the thing being measured, which is why the
results table in chapter 16 says in bold that its cells compare down a column
and never across one.

## Say it like this

> Image-based visual servoing defines the error directly in the image: where
> the markers are versus where they should be, in normalised camera
> coordinates. You never estimate a 3D pose. The loop is: detect, subtract,
> build the matrix that maps camera motion to image motion, invert it, command
> the motion. The correspondence step — which detected dot is which — is the
> part that looks trivial and isn't.
