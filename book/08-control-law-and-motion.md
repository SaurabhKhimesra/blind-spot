# 8. The control law, and turning it into motion

## Deriving the law

We want the image error to die away exponentially (chapter 5):

```
ė = −λ e
```

We know how the image moves when the camera moves (chapter 7):

```
ṡ = L v      and since s* is fixed,     ė = ṡ = L v
```

Set them equal and solve for the command:

```
L v = −λ e        →        v = −λ · L⁺ e
```

That is classic IBVS in one line, and it is `run_ibvs()` in `ibvs_core.py`.

## What the pseudo-inverse is doing here

With six markers, `L` is 12×6: twelve equations, six unknowns. No `v` makes
every equation exactly true, so `L⁺` returns the **least-squares** answer, the
motion that comes closest to the requested image change.

With two markers, `L` is 4×6: four equations, six unknowns. Now many motions
fit exactly, and `L⁺` returns the **minimum-norm** one — the smallest.

Remember that difference. In chapter 12 it is the entire reason classic IBVS
survives with two markers while the 2001 partition explodes: one is protected
by the minimum-norm rule, and the other has been rearranged into an exactly
determined 4×4 with no slack left.

## Why this is not the same as "move toward the target"

A natural but wrong mental model is that the controller looks at the dots,
notices they are up and to the left, and moves up and to the left. It does
something subtler: it asks for the *fastest reduction of the image error in a
least-squares sense*, and any camera motion that achieves that is acceptable
to it.

If backing away happens to make all twelve numbers smaller more efficiently
than rotating does, the controller backs away. That is not a bug in the
implementation; it is the exact solution to the problem as posed. Chapter 10
is what that looks like when it goes wrong: the camera flies 68 metres
backwards while the error decays beautifully the whole time.

## From twist to motion: integrating

The command is a velocity; the simulation needs a pose. Apply it for one step
with the matrix exponential (chapter 2):

```
cTo ← se3_exp(−v · dt) · cTo
```

On a real robot this step is not free, and chapter 24 does it properly: the
camera twist is mapped through the arm's geometric Jacobian into joint
velocities, clipped to the joints' limits, integrated for one cycle, and sent
to a joint trajectory controller.

## The loop, in code

```python
for k in range(steps):
    P_c = transform_points(cTo, P_o)     # markers in the camera frame
    Z   = P_c[:, 2]
    s   = project(P_c)                   # x = X/Z, y = Y/Z
    e   = (s - s_star).reshape(-1)
    L   = interaction_matrix(s, Z)
    v   = -lam * pinv(L) @ e
    cTo = se3_exp(-v * dt) @ cTo
```

Ten lines, and every failure in part 2 happens inside them. The variations
that follow — truncation, the partition, the switch — change exactly one of
these lines each.

## Tuning knobs, and which ones matter

- **λ (gain).** Sets the speed. λ = 0.5 throughout the study.
- **dt.** 0.033 s in the study, 0.1 s on the arm.
- **depth mode.** Affects the scale of the translation columns.
- **τ (truncation).** Introduced in chapter 15, the one constant with a
  documented failure case of its own.

What is *not* a knob: the geometry. When the markers stop carrying the
information the controller needs, no setting of λ, dt or depth recovers it.
That is what makes the failures in part 2 interesting rather than a tuning
exercise.

## Say it like this

> Ask for exponential decay of the image error, substitute the interaction
> matrix, and solve: v equals minus gain times the pseudo-inverse of L times
> the error. The pseudo-inverse is least-squares when you have plenty of
> features and minimum-norm when you don't, and that distinction decides who
> survives the two-feature case. Integrating the twist onto the pose uses the
> matrix exponential, and on the real arm it goes through the robot's Jacobian
> instead.
