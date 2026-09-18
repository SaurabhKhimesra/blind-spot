# 1. The question

## In plain words

Factories use robots that find things with a camera and move to them. The
camera tells the robot "the part is over there", the robot moves, the camera
looks again, and so on, many times a second.

This project asks one narrow question about that loop: **what does the robot
do in the split second when the camera stops being able to tell it what it
needs?** Not "does it recover eventually" — what does it do *right then*. The
answer, for the standard method, is that it speeds up rather than slowing
down, and that is how parts get bent.

## Where it started

The starting question was narrow and practical: **when a vision-guided robot
loses sight of what it is aiming at, what does the controller do in the next
few milliseconds?**

Not "does it eventually recover" — the interesting part is the transient. A
robot closing the last few centimetres onto a part is in the worst place for
its own perception: the part fills the view, markers leave the frame, the
geometry it triangulates against flattens out. If the controller reacts to
that by *lurching*, the cost is not an abstract instability, it is a bent part
or a safety stop that shuts the cell down until an operator resets it.

The founding hypothesis was a machine-learning one: *a learned policy is
needed to decide, per step, which control law to run.* That hypothesis is
dead, and the project is more useful because it died. Chapter 8 has the
number that killed it.

## The ladder

The work was built as a ladder of rungs, each a working controller that the
next rung had to beat:

1. **Classic IBVS** — the textbook law. Establishes the failures.
2. **Partitioned IBVS (2001)** — the published fix for the most famous
   failure. Does it survive the others?
3. **Adaptive-rank truncation** — the standard numerical answer to an
   ill-conditioned inverse. Does *it* survive the others?
4. **The switch** — the partition as a runtime decision rather than a fixed
   choice, with six candidate rules for making that decision.
5. **The guard** — the one rule that shipped, packaged as a library with a
   calibration procedure, a ROS 2 node, and a Gazebo arm demo.

Each rung is a file in the repo, in that order: `ibvs_core.py`,
`partitioned.py`, `truncated.py`, `switched.py`, `blindspot/`.

## How the scope was fixed

Three decisions kept it from sprawling, and all three are worth defending:

**No learning until a hand-written rule fails.** The bar was set *before*
running anything: a hand-written rule had to get the two-feature velocity
spike under **2×** for the learned-policy plan to be abandoned. It reached
**0.5×**. Setting the bar first is what makes that a result rather than a
rationalisation.

**Every claim is judged on a measure chosen for that failure, and the measures
differ.** Camera retreat is judged on whether the run converges; the dropout
and two-feature cases on the peak commanded velocity inside the degradation
window. Mixing them up produces nonsense, which is why the results table says
in bold that cells compare down a column and never across one.

**Negative results ship.** Seven claims died. They are in the README under
"Dead claims", each with the measurement that killed it, because the trail is
stronger evidence than the result.

## What the project is not

- It is not a claim to have invented a fix for camera retreat. That was solved
  in 1999 (2½D visual servoing) and 2001 (partitioned IBVS). Retreat is used
  here as a **positive control**: a failure with known ground truth, to show a
  health signal fires on a real degeneracy rather than at random.
- It is not hardware. It is a free-flying virtual camera, then a rendered
  camera with a real detector, then a simulated UR5e. Real hardware is the
  open question, and the README says so in the first paragraph.
- It is not a controller. The shipped library decides *which* law to run. It
  does not supply a law for geometry that carries no information; chapter 6
  and the "observability" section of the README are explicit that some
  configurations are unfinishable by any controller.

## Say it like this

> I wanted to know what a vision-guided controller does in the moment it loses
> the geometry it depends on. I started from the assumption that choosing the
> right control law per step needed a learned policy, set a bar in advance for
> what a hand-written rule would have to achieve to kill that idea, and the
> hand-written rule beat the bar by four times. So the deliverable became a
> small runtime check and an honest map of when it works and when it doesn't.
