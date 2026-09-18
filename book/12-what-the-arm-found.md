# 12. What the arm found

## In plain words

The point of putting it on a robot was to see whether anything broke that a
free-flying camera could not have shown. Two things did, and both changed what
the README claims.

The first: the guard **cried wolf**. A perfectly healthy panel, tilted so the
camera sees it at a steep angle, looks exactly like a collapsing one — because
both make the marker patch smaller. The second, found while checking a
sentence I had written about noise: the benchmark that made the guard look
best is set up in a way that hides a real cost.

## Finding 1: the false positive

The first Gazebo demo tilted the panel toward edge-on mid-approach. The
guarded cell dropped the partition at margin 0.55× and held its standoff; the
unguarded one kept the partition and drove in close.

That looks like a win, and it is not. The tilted panel is **healthy**. Nothing
about its geometry is degenerate; it is just being viewed obliquely. The guard
fired on a target that was fine.

Reproduced in numpy, on the ring target:

| | area margin | σ₆ margin |
|---|---|---|
| healthy, face-on | 1.69× | 1.88× |
| healthy, 70° | **0.99× — fires** | 27.01× — silent |
| healthy, 75° | **0.86× — fires** | 28.72× — silent |
| collapsed in 3D, c = 0.5 | 1.19× — misses | **0.74× — fires** |

And the physics runs the way chapter 2 predicts: a face-on planar target is
the *worst*-conditioned case, so tilting it **improves** the matrix — σ₆ rises
about 15× — while the projected area shrinks. The two signals move in opposite
directions on the same event.

This is why dead claim 4 was revived. My four-case test suite contained no
case that separates the two signals, so "they matched everywhere" was a fact
about my test set, not about the signals.

## The claim I got wrong, and retracted

In the same run, the guarded cell showed a velocity spike of 2.561 while the
unguarded one stayed at 0.526. I wrote down that the switch itself caused it:
"a pure discontinuity between two control laws".

Then I measured it. Sixteen tilt onset/speed combinations in numpy: the guard
fires every time, and **peak commanded velocity is identical under
partition-always, area guard and σ₆ in 15 of the 16** (the 16th differs by
5%). With little error left to correct, the two laws compute nearly the same
twist, so switching between them costs almost nothing.

The arm spike never reproduced. The most likely cause is perception noise at a
grazing view, where the dots are thin ellipses. The retraction is written into
the file where the wrong claim was.

Worth saying plainly in an interview: **the attribution was too quick, and the
measurement was what caught it.** That is the same discipline as chapter 7,
applied to my own conclusion.

## Finding 2: the benchmark's symmetric start

This one came from checking a sentence, not from looking for a bug. The README
said the area guard is "more noise-robust" than σ₆. The statistic part was
measured; the comparison to σ₆ never had been. Measuring it turned up
something bigger.

The retreat benchmark starts from an **exact** 180° rotation about the optical
axis. In that exact configuration, the partition's sideways command is zero by
symmetry — measured 2.1e-15, i.e. numerically zero.

Start it 2 cm off, or 3° off, and the symmetry breaks. All of that sideways
command then lands in the two weakest directions of `L_xy`, about **130×**
below the strongest. Those two directions are an **orbit about the target**
(`vx = −Z·ωy`, `vy = +Z·ωx`) — the motion that barely changes the image, which
is exactly why they are weak and exactly why the pseudo-inverse amplifies
them.

The camera swings nearly edge-on. Over 100 random starts per angle (lateral
≤ 2 cm, tilt ≤ 3° per axis, exact features), at 180°:

| | switched | converged | lost target edge-on | furthest | peak tilt median/max |
|---|---|---|---|---|---|
| partition always on | 0 | 97 | **3** | 0.80–0.81 m | 84° / 89° |
| σ₆ guard | 0 | 97 | **3** | 0.80–0.81 m | 84° / 89° |
| area guard | 100 | **100** | 0 | **2.02–2.88 m** | 76° / 78° |

Read that carefully, because it is a trade and not a verdict:

- The **area guard** fires in all 100 runs. It is wrong about *why* the view
  is shrinking, and the law it hands control to is the one that retreats, so
  the camera backs off to 2–2.9 m instead of 0.80 m. But it never lost the
  target.
- **σ₆** is right that the target and the matrix are healthy, so it never
  switches — and therefore inherits every orbit, including the three that go
  fully edge-on and lose the target.

Calibration cannot buy both. Widening the healthy tilt range the threshold is
drawn from shrinks the retreat (2.59 m → 1.26 m at 0.9 rad) until, at 1.2 rad,
the guard stops firing at all and loses the target alongside the partition —
and the same widening blinds it further to a mild collapse (1.19× → 2.46×).

And 0.5 px of detector noise is enough to trigger the whole thing from the
exact start: the area guard drops the partition for 123–125 steps and backs
away to 1.80–2.03 m across five noise seeds, while σ₆ does not switch.

## What changed in the repo because of this

- Dead claim 4 was rewritten twice, with both tables.
- The results table gained a note that its retreat start is an **exact**
  symmetric special case, and what happens from an imperfect one.
- The noise paragraph now says the steadier *statistic* did not produce the
  steadier *decision*, with both numbers.
- Test sections 20 and 21 lock all of it, including the orbit mechanism: that
  100% of the commanded sideways motion lies in the two weak directions, and
  that those directions are the orbit at the target's depth.

## Say it like this

> Putting it on the arm found two things the free-flying study could not. One:
> the guard fires on a healthy part seen at a steep angle, because obliquity
> and degeneracy shrink the marker patch identically — and the spectral signal
> moves the *other* way on the same event, which is what revived a claim I had
> killed. Two: the benchmark that made the guard look best starts from an
> exactly symmetric pose, and from a start 2 cm off, the partition's own solve
> lands in a direction the image can barely see and swings the camera edge-on.
> The guard catches that and pays for it with a two-metre retreat; the
> spectral rule stays silent and loses the target in 3 runs out of 100.
> Neither dominates, and the README says so.
