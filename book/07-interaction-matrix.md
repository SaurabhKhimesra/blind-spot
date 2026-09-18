# 7. Deriving the interaction matrix

This is the one derivation to know by heart. Everything in parts 2 and 3 is a
consequence of the shape of this matrix.

## The question

*If the camera moves with twist `v = (vx, vy, vz, ωx, ωy, ωz)`, how fast does
a marker move in the image?*

Answer it and you have the rule linking action to observation, which is
exactly what a controller needs to invert.

## Step 1: how the point moves in the camera's frame

A marker is fixed in the world; the camera moves. So in the **camera's** frame
the point appears to move backwards. From chapter 2:

```
ṗ = −v_lin − ω × p           with p = (X, Y, Z)
```

Writing the cross product out:

```
Ẋ = −vx − (ωy·Z − ωz·Y)
Ẏ = −vy − (ωz·X − ωx·Z)
Ż = −vz − (ωx·Y − ωy·X)
```

## Step 2: how the image coordinate depends on the point

From chapter 3, the projection is `x = X/Z`. Differentiate it with the
quotient rule:

```
ẋ = (Ẋ·Z − X·Ż) / Z²  =  Ẋ/Z − (X/Z)·(Ż/Z)  =  Ẋ/Z − x·Ż/Z
```

The same for `y = Y/Z`:

```
ẏ = Ẏ/Z − y·Ż/Z
```

## Step 3: substitute and collect

Take the `ẋ` equation and put Step 1 into it.

```
Ẋ/Z   = (−vx − ωy·Z + ωz·Y) / Z
      = −vx/Z − ωy + ωz·y                      (using Y/Z = y)

x·Ż/Z = x·(−vz − ωx·Y + ωy·X) / Z
      = −x·vz/Z − x·y·ωx + x²·ωy               (using X/Z = x)
```

Subtract the second from the first:

```
ẋ = −vx/Z + x·vz/Z + x·y·ωx − (1 + x²)·ωy + y·ωz
```

And by the identical route for `y`:

```
ẏ = −vy/Z + y·vz/Z + (1 + y²)·ωx − x·y·ωy − x·ωz
```

## Step 4: write it as a matrix

Those two lines are linear in the six components of `v`, so pull the
coefficients out into rows:

```
ẋ = [ −1/Z,   0,   x/Z,    x·y,   −(1 + x²),   y ] · v
ẏ = [   0,  −1/Z,  y/Z,  1 + y²,    −x·y,     −x ] · v
```

Stack one such pair per marker and you have the **interaction matrix** `L`,
shape `2N × 6`, with

```
ṡ = L v
```

That is `interaction_matrix()` in `ibvs_core.py`, and every entry is checked
by finite difference in `fd_check.py` — nudge the camera, measure where the
dots actually went, compare. A derivation you have checked against the
simulator is worth more than one you have checked against a paper.

## Reading the matrix

Four things are visible by inspection, and all four become chapters later.

**Depth appears only in the translation columns.** `1/Z` sits under `vx, vy,
vz` and nowhere else. So rotation can be controlled without knowing depth;
translation cannot. Every "how far away am I" problem in this book traces back
to this.

**Translation effects shrink with distance, rotation effects do not.** At
Z = 2 m the `vx` column is half what it is at 1 m, while `−(1 + x²)` is
unchanged. Far away, the camera can rotate cheaply and must translate a long
way to achieve anything.

**Sliding and rotating look alike.** Compare the `vx` column, `−1/Z`, with
the `ωy` column, `−(1 + x²)`. For a small target near the image centre, `x` is
small, so one is roughly `−1/Z` and the other roughly `−1`. They differ only
by a scale — which means a *combination* of them produces almost no image
motion at all. That combination is the near-null direction of chapters 4 and
25, and it works out to `vx = −Z·ωy`: the camera **orbiting** the target.

**A flat target seen face-on is the worst case.** Every marker then has the
same `Z`, so the whole `vz` column is proportional to the position `x`, which
is also what a rotation does. Depth motion and rotation become hard to
distinguish, the sixth singular value gets small, and — counter-intuitively —
**tilting the target improves conditioning**, because it spreads the depths
out. That single fact is why a signal computed from the matrix and a signal
computed from the picture disagree about a tilted part (chapter 25).

## Where `Z` comes from

`L` needs a depth per marker, and a single camera does not measure it.
Implemented options (`depth_mode` in `run_ibvs`):

- `true` — exact depth, simulation only, used as the control condition.
- `desired` — the depth at the goal pose. Standard practice.
- `constant` — one number for every marker. What the Gazebo arm uses
  (0.26 m), which is what a real cell with no depth sensor would do.

Errors in `Z` scale the translation columns. The controller still converges
because the direction of the correction stays roughly right; the speed is off.
It is a remarkably forgiving dependency, which is one of the reasons IBVS is
popular.

## Say it like this

> You differentiate the projection x = X/Z, substitute the rigid-body velocity
> of a point seen from a moving camera, and collect terms. Out comes two rows
> per feature, six columns, mapping camera twist to image velocity. Reading
> those rows tells you everything that goes wrong later: depth only enters the
> translation columns, and the sideways-translation column and the
> rotation-about-vertical column are nearly parallel, which is the weak
> direction that later swings the camera edge-on.
