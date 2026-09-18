# 5. Truncation and the switch

## In plain words

If part of your information is missing, the honest thing is to not act on that
part. **Truncation** is exactly that. When you invert the matrix, you look at
its six "directions" and how strong each one is. Any direction that has gone
too weak, you skip — the robot simply doesn't move that way this step.

That's the numerical fix. The other fix is the **switch**: run the 2001
partition when it is safe, and plain truncated IBVS when it is not, deciding
fresh every single step. The rest of the project is about what to base that
decision on.

## Adaptive-rank truncated pseudo-inverse

`truncated.py`. Take the SVD of `L`, keep only singular values above a
*relative* threshold, invert those, zero the rest:

```python
U, S, Vt = svd(L)
keep = S > rel_tau * S[0]        # relative to the strongest direction
```

`rel_tau` (τ) is the one tuning constant in the whole controller, and the
default is **1e-3**, chosen from a sweep (`tau_sweep.py`).

Truncation fixes the dropout spike (**28.7× → 0.9×**) and does nothing at all
for camera retreat (**13.77 m, still diverging**), which makes sense: retreat
is a coupling problem, not a conditioning one, and truncation only helps with
conditioning.

τ has a documented failure of its own, in the README's limits: on a collapsed
target with an initial rotation about the collapse line, the switched
controller at τ=1e-3 **stalls at 0.297 m of pose error while plain classic
IBVS converges exactly**. Truncation is discarding a direction that is weak
but still usable. τ=1e-5 completes the same case exactly. So the shipped
default has a constructible case where intervening is worse than doing
nothing. That is in the repo, in bold, because a single global constant chosen
from one geometry is exactly the kind of thing that breaks on someone else's.

## The switched controller

`switched.py`. Every step: build `L`, ask a rule whether the partition is
safe, and run the corresponding law.

```python
if rule_says_safe(L, L_xy, s_visible):
    v = partitioned_law(...)        # 2001 partition + truncated reduced inverse
else:
    v = -lam * pinv_truncated(L, tau) @ e
```

Six rules were implemented and compared, not one:

| rule | what it looks at |
|---|---|
| `count` | how many features are visible |
| `rank_margin` | numerical rank of `L` versus the unknowns in `L_xy` |
| `sigma6` | the 6th singular value of `L`, one global threshold |
| `sigma6_n` | the same, with the threshold conditioned on the visible count |
| `area_guard` | the polygon area — the partition's own depth feature |
| `alpha_guard` | the image distance between the two features alpha uses |

The point of implementing all six is that the eventual claim is comparative.
"The area guard is good" is not a result; "the area guard matched the spectral
rule on all four benchmark cases, beat it on one, and here is the case where it
is worse" is.

## The results table

| Controller | Retreat 180° | Dropout spike | 2 features | Collapsed target | Clean cost |
|---|---|---|---|---|---|
| Classic IBVS | ✗ 68.19 m, diverges | ✗ 28.7× | ✓ 0.9× | ✓ conv, 8e-11 | baseline |
| Partitioned (2001) | ✓ 0.80 m, converges | ✓ 1.1× | ✗ 288.4× | ✗ floors at 7.3e-3 | +13% |
| Adaptive-rank truncation | ✗ 13.77 m, diverges | ✓ 0.9× | ✓ 0.9× | ✓ conv, 2.4e-5 | baseline |
| Both combined (fixed) | ✓ 0.80 m, converges | ✓ 1.1× | ✗ 11.6× | ✗ floors at 2.0e-3 | +13% |
| Switched, feature count | ✓ 0.80 m, converges | ✓ 1.1× | ✓ 0.5× | ✗ floors at 2.0e-3 | +13% |
| Switched, σ₆ spectrum | ✓ 0.80 m, converges | ✓ 1.1× | ✓ 0.5× | ✓ conv, 2.4e-5 | +13% |
| **Switched, area guard** | **✓ 0.80 m, converges** | **✓ 0.5×** | **✓ 0.5×** | **✓ conv, 2.4e-5** | **+13%** |

Read it down a column, never across one. Retreat and collapse are judged on
**convergence** (final error below 1e-4). Dropout and two-features are judged
on the **velocity spike**: the peak commanded speed during the degraded window
divided by that same controller's speed just before it. Those are different
measures because the failures are different, and a single score would hide
exactly the thing being measured.

The two bottom rows are the result: **switching closes the 11.6× gap to 0.5×**
and keeps everything the fixed combination had. The gap between the last two
rows — feature count versus a geometry signal — is the collapsed-target
column, where counting sees six healthy features and fails.

## What a "0.5× spike" means

Lower than 1 means the controller moved *slower* during the disruption than
before it. That is the desired behaviour: when information goes missing, do
less, not more. A spike above 1 means the controller sped up in response to
losing information, which is the lurch this whole project is about.

## Say it like this

> Truncation is the standard numerical answer: drop the directions that have
> gone blind. It fixes the dropout spike and does nothing for retreat, because
> retreat isn't a conditioning problem. So I made the 2001 partition a runtime
> decision instead of a fixed choice, implemented six candidate rules for
> making that decision, and compared them on all four failures with the
> measure appropriate to each. Switching closes the worst gap from 11.6× to
> 0.5×, and the interesting difference between the rules shows up only on the
> case where feature counting is blind.
