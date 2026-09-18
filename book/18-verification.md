# 18. How correctness was established

## In plain words

In this kind of work, the code usually runs fine while being completely wrong.
A flipped sign doesn't crash anything; it just makes the robot turn the wrong
way, and then you spend a day blaming the control theory.

So the rule for the whole project was: **don't reason about whether the maths
is right, measure it.** If a formula says "when the camera rotates this way,
the dot moves that way", then nudge the camera a tiny bit in simulation, see
which way the dot actually moved, and compare. That's a finite-difference
check, and it caught two sign bugs that reading the code did not.

## Finite differences: `fd_check.py`, 16 checks

The idea: a derivative is a prediction about what happens under a small
change. So make the small change and compare.

```
predicted = L @ v            # what the interaction matrix says will happen
actual    = (features after moving by v·h − features before) / h
```

If those disagree beyond numerical noise, something is wrong — and it doesn't
matter whether the mistake is in the formula, a sign convention, or the frame
you assumed.

The 16 checks cover every derivative and sign the project relies on: each row
of the interaction matrix, the derivative of the polygon area feature, the
derivative of the line-angle feature (including the exact `d(alpha)/d(wz) = −1`
that fixes the sign in the partitioned law), `se3_exp` and `se3_log`, and the
null spaces claimed for degenerate configurations.

**Two sign bugs in `partitioned.py` were found this way, and neither would
have been caught by inspection.** That sentence is in the README's reproduce
section as a standing warning to future-me.

## The regression suite: `tests.py`, 177 checks

Every result that took work to establish is locked by a test, so that a later
"harmless" change cannot quietly undo it. The suite is organised in numbered
sections that mirror the story:

| section | what it locks |
|---|---|
| 1 | the maths: projection, interaction matrix, SE(3) |
| 2–5 | the four failures with their measured numbers, and that truncation is free when nothing is degraded |
| 6 | the switched controller and that it doesn't chatter |
| 7–9 | σ₆ as a health metric, the fourth case, and that the spectral rule doesn't lose the first three |
| 10–13 | the area guard, where the two signals differ, count-conditioned calibration, the τ artifact |
| 14–15 | flips under feature noise; hysteresis tested and rejected |
| 16 | the calibration trap |
| 17 | forward prediction (dead claim 6) |
| 18 | where the controller stops and observability begins |
| 19 | the shipped τ's constructible worse-than-nothing case |
| 20–21 | obliquity vs degeneracy, and the retreat row from an imperfect start |
| 22 | the folding part, using the arm's own control law |

Plus `test_blindspot.py` (34 checks) for the packaged API and the ROS bridge —
including tests that the library **refuses** to do things: no default
threshold, no guessing units, no accepting a 2-feature polygon as healthy.

A test suite that only checks successes is half a suite. Several of these
check that a wrong thing stays wrong: that hysteresis still costs what it
cost, that the in-situ calibration still disables the guard, that the τ case
still stalls.

## The discipline behind the numbers

**Fix the seed, report the sampling.** Thresholds are the healthy 1st
percentile over 4000 random poses with seed 0. Anyone can rerun it and get the
same number.

**Define the measure before quoting it.** "Clean cost +13%" is meaningless
until you say "steps to drive the error below 1% of its initial value"; the
same comparison gives +2.4% at a 50% threshold.

**Report the operating point, not a convenient one.** An earlier version of
the correlated-noise table quoted the *healthy-phase* margin (1.13×) as though
it were the margin during the degradation, which made every row look identical
because it was the same quantity measured on the same pre-degradation
trajectory. The corrected table reports the margin at the count actually seen
that step (1.62×). That correction is written into the README rather than
silently fixed.

**When a result surprises you, suspect the instrument first.** Three of the
five rendered-camera bugs in chapter 20 were found because a number was *too
good*, not too bad.

## Say it like this

> I verified derivatives by finite difference instead of reasoning about sign
> conventions, because two sign bugs in the partitioned law had already hidden
> from inspection. Everything that took effort to establish is locked by a
> regression test — 177 of them, plus 34 on the packaged API — including tests
> that assert the things that *didn't* work still don't, so a future change
> can't quietly revive a claim I killed.
