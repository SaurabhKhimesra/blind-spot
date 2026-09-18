# 10. Failure 1: camera retreat

## In plain words

Ask the controller to do something simple: keep the camera where it is, but
rotate it half a turn about the direction it is looking. Like turning a photo
upside down without moving.

It flies the camera **68 metres backwards** and never comes back.

Nothing is broken. No matrix is singular, no marker is missing, the detector
is perfect. The controller is solving exactly the problem it was given, and
the answer to that problem is insane.

## Why it happens

Recall what the law actually optimises (chapter 8): the motion that reduces
the twelve image-error numbers fastest, in a least-squares sense. It does not
care how it gets there.

Now picture the markers as a hexagon in the image that has to end up rotated
by 180°. Two motions reduce the error:

1. **Rotate** about the optical axis. Every marker travels along a circular
   arc — a long way around — before anything lines up.
2. **Back away.** Everything shrinks toward the image centre, which reduces
   all twelve numbers immediately, and it costs nothing in the first instant.

Locally, option 2 is the better bargain, so the least-squares solution takes
it. And it keeps taking it, because the trade never reverses: the further back
you go, the smaller the errors, and the rotation still has not happened. The
camera commits to a spiral it never finishes.

This is a **coupling** failure. It is not a numerical one. Nothing about the
matrix is ill-conditioned at any point on that trajectory, so no amount of
truncation, regularisation or careful inversion helps — measured: adaptive-rank
truncation still diverges, at 13.77 m.

## The measurement that changed the project

Throughout the retreat, compute the alignment between the commanded motion and
the error — the cosine of the angle between them, which says "is the
controller pushing in the right direction?"

It is exactly **−1.0000** at every single step, while the camera flies 68 m
away.

By its own arithmetic, the controller is perfect. The error is decaying
smoothly. Every quantity a naive monitor would watch says everything is fine.

**So image error is a useless health signal.** Anything that watches the error
reports perfect health during the most spectacular failure in the book. That
finding decided the design of everything afterwards: every candidate health
signal in this project is a property of the *geometry the controller depends
on*, never of its error.

## Why it is in the project at all

Retreat was solved before this project started — by 2½D visual servoing
(Malis, Chaumette & Boudet, 1999) and by the partitioned scheme (Corke &
Hutchinson, 2001), which is the one studied here.

So it is not a contribution. It is a **positive control**: a failure with
known ground truth. If a proposed health signal does not fire on retreat, the
signal is useless; if it fires there and nowhere sensible, it is a smoke
alarm that only rings at parties. Having one failure with a known answer is
what makes the other three interpretable.

## The numbers

| controller | furthest distance | outcome |
|---|---|---|
| classic IBVS | 68.19 m | diverges |
| adaptive-rank truncation | 13.77 m | diverges |
| partitioned (2001) | 0.80 m | converges |
| combined, and every switched rule | 0.80 m | converges |

0.80 m is the starting distance, so "0.80 m" means the camera never backed up
at all.

## The catch, discovered much later

That benchmark starts from an **exactly** symmetric pose: a pure 180° rotation
about the optical axis with no offset. In that configuration the partitioned
law's sideways command is zero by symmetry — measured 2.1e-15, numerically
zero.

Start it 2 cm or 3° off, and the symmetry breaks. What happens then is
chapter 25, and it is the most interesting result in the second half of this
book: the camera swings nearly edge-on, one guard catches it and pays for it
with a 2–3 m retreat, and the other stays silent and loses the target
entirely in 3 runs out of 100.

A benchmark that starts from a perfectly symmetric pose is not lying, but it
is answering an easier question than the one you think you asked.

## Say it like this

> The classic failure: ask for a 180° rotation about the optical axis and the
> camera flies 68 metres backwards, because retreating reduces the image error
> faster than rotating does and the least-squares solution takes that bargain.
> It's a coupling problem, not a conditioning one, so truncation doesn't help.
> The part I took from it is that the controller's own error signal reads
> minus one — perfect alignment — the entire time, so error is useless as a
> health measure.
