# 16. The switch: deciding per step

## In plain words

Two good control laws, each wrong in a different situation. The obvious move
is to stop choosing once, at design time, and choose every step instead —
using the partition while it is safe, and the plain truncated law while it is
not.

That is the switch. The rest of the project is about what to base that
decision on, and how to know the decision is right.

## The controller

```python
if rule_says_safe(L, L_xy, s_visible):
    v = partitioned_law(...)        # 2001 partition + truncated reduced solve
else:
    v = -lam * pinv_truncated(L, tau) @ e
```

Decided fresh every step, from the current image alone. No timers, no modes,
no memory: if the situation improves, the partition comes back the same way it
left.

## Six candidate rules, not one

| rule | what it watches |
|---|---|
| `count` | how many markers are visible |
| `rank_margin` | numerical rank of `L` versus the unknowns in `L_xy` |
| `sigma6` | the 6th singular value of `L`, one global threshold |
| `sigma6_n` | the same, threshold conditioned on the visible count |
| `area_guard` | the polygon area — the partition's own depth feature |
| `alpha_guard` | the image distance between the two markers `alpha` uses |

Implementing all six is what makes the eventual claim comparative. "The area
guard works" is not a result. "The area guard matched the spectral rule on all
four benchmark cases, beat it on one, and here is the case where it is worse"
is a result.

Two of these died on measurement and are in chapter 19: `rank_margin` because
at two features the reduced matrix is 4×4 but numerically rank 3, so the rule
kept the partition on in exactly the case where it must go; and the idea of a
single global spectral threshold, because a healthy target's weak directions
and a collapsing target's overlap, pose-dependent, by a factor of 1.65–3.73.

## The results

| Controller | Retreat 180° | Dropout spike | 2 features | Collapsed target | Clean cost |
|---|---|---|---|---|---|
| Classic IBVS | ✗ 68.19 m, diverges | ✗ 28.7× | ✓ 0.9× | ✓ conv, 8e-11 | baseline |
| Partitioned (2001) | ✓ 0.80 m | ✓ 1.1× | ✗ 288.4× | ✗ floors 7.3e-3 | +13% |
| Adaptive-rank truncation | ✗ 13.77 m | ✓ 0.9× | ✓ 0.9× | ✓ conv, 2.4e-5 | baseline |
| Both combined (fixed) | ✓ 0.80 m | ✓ 1.1× | ✗ 11.6× | ✗ floors 2.0e-3 | +13% |
| Switched, feature count | ✓ 0.80 m | ✓ 1.1× | ✓ 0.5× | ✗ floors 2.0e-3 | +13% |
| Switched, σ₆ spectrum | ✓ 0.80 m | ✓ 1.1× | ✓ 0.5× | ✓ conv, 2.4e-5 | +13% |
| **Switched, area guard** | **✓ 0.80 m** | **✓ 0.5×** | **✓ 0.5×** | **✓ conv, 2.4e-5** | **+13%** |

Read it **down a column, never across one**: retreat and collapse are judged
on convergence, the other two on the velocity spike (chapter 9). A single
combined score would hide exactly what is being measured.

What the table says:

- Switching **keeps everything** the fixed combination had and closes the
  two-feature gap from 11.6× to **0.5×**.
- The only column that separates the switching rules is **collapsed target**,
  where counting features sees six healthy markers and fails while both
  geometric signals succeed.
- The cost of switching is zero in the clean case: +13% is the partition's
  own cost, paid by every row that uses it.

## What this killed

The project's founding hypothesis was that this decision needed a learned
policy, with a bar set in advance at 2×. A one-line comparison reached 0.5×,
so the learning plan was abandoned before any data was collected.

That is worth saying carefully in an interview, because it sounds like an
excuse for not doing the hard thing. It is the opposite: the hard thing was
building four honest failure cases and six candidate rules, which is what made
it possible to *know* that the simple answer was sufficient. Without the
comparison you could ship a learned policy and never find out it was
unnecessary.

## Say it like this

> Instead of choosing a control law at design time, choose every step: run the
> 2001 partition when it's safe and the plain truncated law when it isn't. I
> implemented six candidate rules for that decision and compared them on all
> four failures with the measure appropriate to each. Switching closes the
> worst gap from 11.6× to 0.5×, and the only case that separates the rules is
> the one where all six markers are visible and the geometry has gone flat.
