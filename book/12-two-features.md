# 12. Failure 3: two features left

## In plain words

Now hide four of the six markers. Two left.

Classic IBVS shrugs: spike **0.9×**, it moves *more gently* than before. The
published 2001 fix — the good one, the one that solves camera retreat —
commands a motion **288.4 times** larger than it was moving.

Same disruption, same instant, same target. The difference is entirely in the
structure of the two control laws.

## Why the plain law survives

Two markers give four numbers, so four equations. The camera has six degrees
of freedom. Four equations, six unknowns: **under-determined**. Infinitely
many motions satisfy the request exactly.

The pseudo-inverse has to pick one, and it picks the **smallest** (chapter 4).
So when information disappears, classic IBVS is handed the gentlest motion
consistent with what it can still see.

It is not clever. It is not intentional. It is a property of the minimum-norm
solution, and it protects the controller by accident.

## Why the 2001 fix does not

The partition (chapter 14) improves retreat by taking two degrees of freedom
out of the inverse and driving them from direct image measurements. The other
four still come from a reduced matrix, `L_xy`.

Count again with two markers: `L_xy` is **4×4**. Four equations, four
unknowns, exactly determined.

There is now no redundancy to average over and no minimum-norm rule to fall
back on, because there is exactly one solution and the law must take it. If
that 4×4 is ill-conditioned — and with two markers it is — the single solution
is enormous.

**The published fix for failure 1 is the cause of failure 3.** That sentence
is the hinge of the whole project.

## The numbers, and the gap that defines the work

| controller | spike |
|---|---|
| classic IBVS | 0.9× |
| adaptive-rank truncation | 0.9× |
| partitioned (2001) | **288.4×** |
| both combined (fixed) | **11.6×** |
| switched, any guard rule | **0.5×** |

Adding truncation on top of the partition — the "combined" controller —
reduces 288× to 11.6×. Better, and still a lurch: eleven times the previous
speed, at the moment the robot knows least.

**11.6× is the gap.** The founding hypothesis was that closing it needed a
learned policy. The bar for abandoning that plan was set in advance at 2×. A
one-line rule reached **0.5×**.

## Why truncation alone cannot rescue the partition

Worth being precise, because it is a natural objection. Truncation works by
declining to move along directions that have gone weak. But the partition has
already *committed* two degrees of freedom to a separate path — they are
computed from the area and the line angle, not from the inverse — so the four
that remain must be solved exactly as a block. Truncating inside a 4×4 that
has to produce one specific answer either does nothing or removes a direction
the law still needs.

The only real fix is to stop using the partition while the situation lasts,
which is the switch (chapter 16).

## The minimum-feature honesty note

To let a genuine two-feature step happen at all, the simulation has to be told
that two is acceptable (`min_features=2`); by default the harness falls back to
using every marker still available when fewer than three are visible, which is
what a competent implementation would do. Studying the two-feature regime
means deliberately switching that protection off, and saying so.

## Say it like this

> With two markers, classic IBVS is under-determined, so the pseudo-inverse
> hands it the minimum-norm solution and it moves gently: 0.9×. The 2001
> partition has taken two degrees of freedom out of the inverse, so its
> reduced solve is an exactly-determined 4×4 with no slack, and it spikes
> 288×. Adding truncation gets it to 11.6×, and that 11.6 is the number the
> rest of the project exists to close. A one-line runtime rule gets it to 0.5.
