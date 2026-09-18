# 8. Dead claims

## In plain words

This is the chapter to read before an interview. It is a list of things I
believed, wrote down, and then had to take back because a measurement said
otherwise. Each one has the number that killed it.

Keeping this list is not modesty. In work like this, the claims that survived
are only trustworthy if you can see how many didn't.

## 1. "Closing the 11.6× gap needs a learned policy"

*The project's founding hypothesis.*

Killed by a one-line hand-written rule reaching **0.5×**, against a bar of 2×
that was set before running it. No learned policy is needed for this decision.

The reason this is a *useful* negative result: training a policy would have
meant collecting data, defining a reward, and shipping a model that nobody can
inspect — to replace a comparison between two numbers.

## 2. "A rank-margin rule on the spectrum beats counting features"

Killed by finite difference. At two features, `L_xy` is 4×4 but **numerically
rank 3**, so the condition `rank(L_xy) < rank(L)` held and the partition
stayed ON in exactly the case where it must be dropped.

The bug was comparing against the wrong matrix. The fixed rule compares the
numerical rank of the *full* `L` against the number of unknowns in the reduced
solve.

## 3. "A single global τ separates healthy-but-weak from degenerate"

Killed by the measured band. A healthy ring's weak directions sit at
σ₅/σ₁ = **1.4e-3 … 3.1e-3** depending on pose. A *collapsing* target's σ₅ is
already at **8.2e-4** — below healthy. The separating band is a factor of
**1.65–3.73**, and it moves with pose.

So no global threshold is defensible. A rank test has to use an existence
threshold instead, after which it reduces to counting features, which is
already a rule we have.

## 4. "The switching decision needs the interaction-matrix spectrum"

**Killed, then partly revived.** This is the most instructive entry in the
project, because it moved twice.

*Killed:* on all four benchmark cases the area guard matched σ₆ everywhere and
beat it on dropout (0.5× vs 1.1×), with no spectrum at all.

*Revived:* that conclusion was a property of the **test suite**, not of the
signals. None of the four cases separates them. The Gazebo arm work found one
that does — a healthy planar target viewed obliquely:

| | area margin | σ₆ margin |
|---|---|---|
| healthy, face-on | 1.69× | 1.88× |
| healthy, 70° | **0.99× fires** | 27.01× silent |
| healthy, 75° | **0.86× fires** | 28.72× silent |
| collapsed in 3D, c = 0.5 | 1.19× misses | **0.74× fires** |
| collapsed in 3D, c = 0.05 | 0.38× fires | 0.07× fires |

The area signal cannot tell a degenerate part from a healthy one seen at a
steep angle; both shrink the projected polygon. σ₆ can, and in the direction
physics predicts: a face-on planar target is the *worst*-conditioned IBVS case
because every point sits at one depth, so tilting it **improves** conditioning
(σ₆ rises about 15×) while the projected area shrinks.

*And then the cost was measured*, which is chapter 12: on a tilting target the
false positive is free, but on the retreat benchmark from a start just 2 cm or
3° off the exact symmetric one, it costs a 2.0–2.9 m retreat in 100 runs out
of 100 — while σ₆, by staying silent, rides the partition into losing the
target entirely in 3 runs of 100.

Final ledger: neither dominates. σ₆ is the faithful signal of whether the
*matrix* is degenerate; the area is the faithful signal of whether the
*partition's own feature* is healthy, and the folding part (chapter 13) is the
case where that difference saves the run and σ₆ is blind at 1.29×.

## 5. "The collapsed-target velocity spike is evidence for switching"

Killed by τ arithmetic. At τ=1e-3 the truncation threshold sits at **98.78%**
of the smallest singular value, so the spike is a 1.2%-margin artifact: across
the τ sweep, combined's peak falls **5.96 → 0.326** against truncation-only's
**0.322**. The gap essentially vanishes.

Only the *convergence* half of that row is real evidence, and it is
τ-independent from 1e-4 to 1e-1. The results table therefore reports that case
on convergence and says in the caption why.

## 6. "Predicting the guard signal forward buys useful warning"

This one died on structure, not on quality. Rolling the features forward with
`s_next = s + L v dt` predicts the degeneracy the **camera** causes, never the
one the **world** causes:

| transition | example | warning |
|---|---|---|
| camera-driven | classic IBVS retreating | **231 ms** |
| exogenous | an occluder arrives | **0 ms** |
| already degenerate at t=0 | collapsed target | **0 ms** |

The predictor itself is good — one step is sub-pixel (0.157 px median), and
20 steps (660 ms) stays trustworthy even at high speed — and it costs 1.394 ms
per cycle, so the gate passed. It dies because with the partition on, the
camera never retreats, so the area never falls, so **the one case with warning
is the case that never needed it**. And with 231 ms in hand, ramping the gain
down buys 4% on peak velocity and nothing on convergence.

## 7. "The guard's switch caused the spike in the arm demo"

Mine, written into a resume note before it was measured, and retracted the
next day. The tilting-panel arm run showed the guarded cell spiking to 2.561
while the unguarded one stayed at 0.526, and I attributed it to the switch
being "a pure discontinuity between two control laws".

Reproduced in numpy across 16 tilt onset/speed combinations: the guard fires
every time, and **peak commanded velocity was identical under all three rules
in 15 of 16** (the 16th differed by 5%). Switching is nearly free there. The
single arm spike never reproduced and is more likely perception noise at a
grazing view.

The correction is written into the repo where the wrong claim was, not
silently deleted.

## Two further corrections

- **The health metric `S[-1]` was wrong.** It compares matrices of different
  shapes and cannot see a null space: on a two-feature `L` it reads a healthy
  **9.97e-2** where the padded σ₆ is **exactly 0**.
- **The collapsed-target mechanism is not `σ = √area → 0`.** σ and σ* shrink
  together, the ratio holds at **1.4980**, and `vz` is unchanged at every
  collapse level. The failure is in the reduced inverse.

## Say it like this

> Seven claims of mine died in this project, including the founding
> hypothesis, and each one is in the README with the measurement that killed
> it. The most useful one moved twice: I killed the idea that the decision
> needs the matrix spectrum, then the arm demo found a case my four-case test
> suite didn't contain, which partly revived it. That taught me the failure
> wasn't in the signal, it was in my test set.
