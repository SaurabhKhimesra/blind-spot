# 3. How a camera turns the world into numbers

## The pinhole model, derived

Imagine a box with a tiny hole in one face and film on the opposite face.
Light from a point in the world passes through the hole and lands on the film.
That is a **pinhole camera**, and it is the model every equation here uses.

Put the hole at the origin of the camera frame, with `z` pointing out at the
scene, and the image plane at distance `f` in front of it. A point sits at
`(X, Y, Z)`. The ray from the point to the hole hits the image plane at some
`(u, v)`.

Similar triangles do the rest. The triangle from the origin out to the point
has "height" `X` at "depth" `Z`. The triangle from the origin to the image
plane has height `u` at depth `f`. They are the same shape, so

```
u / f = X / Z          →      u = f · X / Z
v / f = Y / Z          →      v = f · Y / Z
```

That is the whole model. Two consequences drive everything later:

- **Everything is divided by depth.** Twice as far away means half the size.
- **Depth is not recoverable from one point.** A point twice as far and twice
  as big lands in exactly the same place. One camera, one point, no depth.

## Pixels, and why this project does not use them

Real sensors count pixels, not metres, and the optical axis rarely lands
exactly at the middle pixel. So the practical version is

```
u_px = fx · (X/Z) + cx
v_px = fy · (Y/Z) + cy
```

where `fx, fy` are focal lengths in pixels and `(cx, cy)` is the **principal
point**, where the optical axis pierces the sensor. Those four numbers are the
camera **intrinsics**, and in ROS they arrive in a `sensor_msgs/CameraInfo`
message.

Divide them back out and you get **normalised image coordinates**:

```
x = (u_px − cx) / fx = X / Z
y = (v_px − cy) / fy = Y / Z
```

This project works entirely in `x, y`. The reason is that they are the
geometry, with the particular sensor divided out — the same target at the same
pose gives the same `x, y` on any camera. That is why `blindspot/units.py`
exists, why it *requires* the principal point rather than assuming the middle
of the image, and why the ROS node insists on `camera_info`.

That "assuming the middle" is not a hypothetical. Chapter 20 is the story of
half a pixel: OpenGL puts the optical axis at `W/2`, OpenCV indexes pixel
centres so the right answer is `(W−1)/2`, and the resulting bias of
(−0.4904, −0.5010) px was hidden for a while by a corner-refinement method
whose own convention cancelled it.

## What the camera can and cannot tell you

For one point you get two numbers, `x` and `y`. The robot has six degrees of
freedom. So:

- **One point** gives 2 constraints. Not enough.
- **Three points** give 6 constraints for 6 unknowns. Just enough, in general.
- **Six points** give 12 constraints for 6 unknowns, comfortably redundant —
  which is why the targets here have six markers and why losing half of them
  is survivable while losing four is not (chapter 12).

"In general" is doing real work in that sentence. Some arrangements of points
carry less information than their count suggests: three points **in a line**
are worth less than three points in a triangle, and there is a specific
surface in space — the **danger cylinder** through three points — where the
geometry is singular no matter how good your camera is. Those cases are in
chapters 13 and 27, and they are the reason this project measures *geometry*
rather than counting features.

## Depth, the quantity nobody has

The projection divides by `Z`, so any controller that reasons about metres
needs `Z`. Options in practice:

1. A depth sensor. Adds hardware and its own failure modes.
2. Estimate it from the target's known size. Needs a model.
3. Use the depth you *expect* at the goal, and accept the error while you are
   far from it. This is standard and it is what the arm demo does (a constant
   0.26 m).
4. Avoid needing it: replace the depth-dependent degree of freedom with
   something you can measure straight off the image. That is exactly what the
   2001 partition does with the polygon area (chapter 14), and its weakness —
   the area can shrink for reasons that are not distance — is the subject of
   this entire book.

## Say it like this

> A pinhole camera divides everything by depth: a point at (X, Y, Z) lands at
> (X/Z, Y/Z) in normalised coordinates, which are pixels with the intrinsics
> divided out. That division is why one camera cannot see depth from one point
> and why every controller here needs either a depth estimate or a substitute
> for it — and the substitute is what fails.
