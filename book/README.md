# The Blind Spot — the whole project, written out

This is the long-form account of the project in `..`: what the problem is, what
was built, every algorithm in it, every number that decided something, and
every problem hit along the way — including the ones where I was confidently
wrong and had to retract in writing.

It exists so the project can be explained out loud, line by line, without
hand-waving.

**Written to be followed by a bright school student.** Every chapter opens
with an **"In plain words"** section that uses no jargon, then gives the
precise version with the real numbers. Every chapter closes with **"Say it
like this"**, a spoken version for an interview.

**Where the ROS 2 work is.** Chapter 10 is the guard as a ROS 2 node inside
someone else's stack; chapter 11 is the full robot stack — Gazebo,
`ros2_control`, bridges, launch, sim time, kinematics — and every problem hit
while building it. Chapter 14 has the ROS questions an interviewer is likely
to ask, and one short section on how to describe the split between the numpy
study and the ROS deployment accurately, because the repo is open and the
split is visible in it.

## Chapters

| # | Chapter | What it covers |
|---|---|---|
| 1 | [The question](01-the-question.md) | Where the project started, the founding hypothesis, how scope was fixed |
| 2 | [Visual servoing from scratch](02-visual-servoing.md) | The algorithm: features, error, interaction matrix, control law, integration |
| 3 | [Four ways to go blind](03-four-failures.md) | The failure modes, what breaks in each, why image error is a liar |
| 4 | [The 2001 partition](04-the-partition.md) | Corke & Hutchinson's fix in full, and the two failures it makes worse |
| 5 | [Truncation and the switch](05-truncation-and-switch.md) | Adaptive-rank pseudo-inverse, τ, the runtime switch, the results table |
| 6 | [The guard](06-the-guard.md) | The shipped signal, calibration, the in-situ trap, noise, hysteresis |
| 7 | [How correctness was established](07-verification.md) | Finite differences, the two sign bugs, the 177-test suite |
| 8 | [Dead claims](08-dead-claims.md) | Seven claims of mine that died, each with the number that killed it |
| 9 | [A real camera and a real detector](09-rendered-camera.md) | MuJoCo + ArUco, the 0.095 px gate, five bugs, correlated degradation |
| 10 | [Packaging it](10-package-and-ros.md) | The library contract, the ROS 2 node, what they refuse to do |
| 11 | [Building the arm environment](11-arm-environment.md) | Gazebo, UR5e, controllers, cameras, kinematics, and every build problem |
| 12 | [What the arm found](12-what-the-arm-found.md) | The false positive, the retracted claim, the retreat-row trade |
| 13 | [The folding part](13-the-folding-part.md) | The case the partition cannot survive, and how the clip was made |
| 14 | [Telling the story](14-interview.md) | 30-second, 2-minute and 10-minute versions, plus a question bank |

## The one-page version

A camera on a robot's wrist steers the robot by watching features on the part.
The standard method, **image-based visual servoing (IBVS)**, has a famous
failure: for a large rotation about the optical axis it flies the camera
backwards instead of rotating. Corke & Hutchinson's 2001 **partitioned**
scheme fixes that by driving the two optical-axis degrees of freedom from
direct image measurements — the area of the feature polygon for depth, the
angle of a line between two features for roll — instead of from the inverted
interaction matrix.

That fix is real, and it makes two *other* failures worse. When features drop
to two, its reduced 4×4 solve becomes exactly determined and brittle
(**288×** velocity spike). When the target's geometry collapses, it forces
four degrees of freedom through an ill-conditioned matrix and floors short of
the goal (**7.3e-3**, never reaching 1e-4).

So the partition should not be a fixed choice. It should be a **runtime
decision**, and the project's founding hypothesis was that deciding it well
needed a learned policy. It does not: a one-line rule on a quantity the
controller already computes closes the gap from **11.6× to 0.5×**, better than
the 2× bar set before running it.

The shipped rule watches the **health of the partition's own substitute
features** — the polygon area against a range calibrated on a known-good
target — rather than the spectrum of the matrix being inverted. That choice is
defensible but not dominant, and the project says so in its own README: the
area signal cannot tell a degenerate part from a healthy one seen at a steep
angle, and on one benchmark start that costs a 2–3 m retreat. The spectral
signal σ₆ is blind to the case a folding part creates. Neither wins outright,
and both failure directions are measured.

Everything is checked twice: derivatives by finite difference rather than by
reasoning about sign conventions (two sign bugs were found that way), results
by a 177-test regression suite, and the control conclusions again against a
**rendered camera with a real OpenCV ArUco detector** (agrees with exact
projection to 0.095 px) and again in a **Gazebo UR5e cell** driving a real
arm's Jacobian.

## How to use this book before an interview

1. Read chapter 14 first. It has the three lengths of the story.
2. Read chapters 2, 4 and 6. That is the actual technical content: the method,
   the fix, and the guard.
3. Skim chapter 8. Interviewers dig for what went wrong; that chapter is
   nothing but what went wrong, with numbers.
4. Keep chapter 11 for the "tell me about a hard debugging session" question.
