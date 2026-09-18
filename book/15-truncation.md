# 15. Truncation: declining to move blind

## In plain words

If part of your information has gone missing, the honest response is to not
act on that part.

Truncation does exactly that. Before inverting the matrix, look at its six
directions and how strongly each one shows up in the image (chapter 4). Any
direction that has gone too weak to trust, skip it — the robot simply does not
move that way this step. You give up control of something you could not see
anyway, and in exchange you are not amplified by one over a tiny number.

## How it works

```python
U, S, Vt = svd(L)
keep = S > rel_tau * S[0]        # relative to the strongest direction
S_inv = where(keep, 1/S, 0)      # weak directions contribute nothing
L_plus = (Vt.T * S_inv) @ U.T
```

Two details that matter:

**The threshold is relative**, not absolute. `rel_tau × S[0]` scales with the
problem, so the same constant means the same thing at different distances and
target sizes. An absolute threshold would need re-tuning for every geometry.

**It is adaptive.** The rank is recomputed every step from the current image,
so the controller uses five directions during an occlusion and six again
afterwards, without any mode switch or timer.

## What it fixes, and what it does not

| failure | truncation |
|---|---|
| feature dropout | **fixes it**: 28.7× → 0.9× |
| two features | fine, 0.9× — but classic IBVS was already fine there |
| collapsed target | **converges**, 2.4e-5 |
| camera retreat | **no help at all**: still diverges, 13.77 m |

The pattern is exact and worth stating: truncation cures **conditioning**
failures and is powerless against **coupling** failures. Retreat has a
perfectly healthy matrix — nothing is weak, so there is nothing to truncate —
and the problem is that the solver's preferred solution is a bad path through
space.

That is why the project needs both ideas rather than one. Truncation handles
the blind-direction failures; the partition handles the coupling failure; and
neither handles what the other does.

## τ: the one constant, and its documented failure

`rel_tau = 1e-3` is the shipped default, chosen from a sweep in
`tau_sweep.py`. It is the only tuning constant in the controller, and the
repository states plainly where it fails.

**The constructible counter-example**: on a collapsed target with an initial
rotation about the collapse line, the switched controller at τ=1e-3 **stalls
at a pose error of 0.297 m**, while plain classic IBVS **converges exactly**
(0.0000 m, image error 1.5e-10). Truncation is discarding a direction that is
weak but still usable, and the guard has handed control to exactly that
truncation.

τ=1e-5 completes the same case exactly, so it is a tuning failure rather than
a structural one — but the shipped default has a case where intervening is
worse than doing nothing, and a single global constant chosen on one geometry
is exactly the sort of thing that breaks on someone else's. The README says:
if you deploy this, sweep τ on your own geometry rather than inheriting 1e-3.

A second appearance of the same sensitivity is dead claim 5 (chapter 19): at
τ=1e-3 the truncation threshold lands at 98.78% of the smallest singular value
on the collapsed-target case, which makes a reported velocity spike an artifact
of a 1.2% margin rather than a property of the controller.

**Any result that moves a lot when τ moves a little is a result about τ.**
That is the rule this project took from it.

## Say it like this

> Truncation filters the pseudo-inverse: run the SVD, drop any direction
> weaker than a relative threshold, invert the rest. It cures the conditioning
> failures — the dropout spike goes from 28.7× to 0.9× — and does nothing for
> camera retreat, because retreat has a healthy matrix and a bad path. The
> threshold is the only tuning constant in the controller and I documented a
> case where the shipped value is worse than not intervening at all.
