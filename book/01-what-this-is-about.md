# 1. What this is about

## The situation

A robot arm has a camera bolted to its wrist. On the part it is working on
there are six small markers. The robot's job is to line itself up with that
part — to plug something in, fit a panel, pick it up — and it does that by
watching the markers and moving until they sit where they are supposed to sit
in the picture.

This is real and ordinary. It is how a lot of vision-guided robots work, and
the method has a name: **image-based visual servoing**, IBVS for short.
"Servoing" just means continuously correcting toward a target.

## The thing that goes wrong

The controller needs to know how far away the part is, and a single camera
cannot measure distance directly. So the well-known fix, published in 2001,
uses a substitute: **how big the patch covered by the markers looks**. Big
patch, close. Small patch, far. Cheap, fast, works.

Now suppose the part folds, or flexes, or is seen from a steep angle. The
patch gets smaller for a reason that has nothing to do with distance. The
controller cannot tell the difference. It concludes "I am too far away" and
drives toward the part — fast, confidently, and wrongly.

An everyday version: judge distance by how big a sheet of paper looks. Someone
folds the paper in half. It looks smaller, so you step toward it. The paper
never moved.

In a factory, that step forward is a bent part, a scrapped assembly, or a
safety stop that shuts down the line until someone walks over and resets it.

## The question this project asks

Not "does the robot recover eventually" — it usually does. The question is
what it does in the **half second** while its information is wrong, because
that is when the damage happens.

The answer, measured: it speeds up rather than slowing down. In one benchmark,
losing four of six markers makes the standard fix command a motion **288 times**
larger than it was moving before. That is not a subtle numerical issue. That
is a robot lunging.

## What was built

Three things, in order:

1. **A study.** All four distinct ways this kind of controller goes blind,
   each reproduced in a small simulation with exact numbers, and every
   candidate fix compared on the measure that suits each failure.
2. **A guard.** One runtime check: watch the very measurement the controller
   leans on, compare it against what that measurement looks like on a
   known-good part, and when it drifts out of range, switch the robot to a
   more conservative control law for as long as it takes. It closes the worst
   gap from 11.6× to **0.5×** — the robot ends up moving *slower* during the
   disruption than before it, which is the behaviour you want.
3. **A robot.** The guard packaged as a ROS 2 node, and a full simulated
   UR5e cell in Gazebo where the loop runs closed: camera → detector → guard →
   control law → the arm's own kinematics, ten times a second.

## What was disproved, including by me

The project started from the assumption that choosing the control law each
step would need machine learning. It does not. A hand-written rule beat the
bar that was set in advance, by four times, and the learning plan was dropped.

Six further claims of mine died the same way, each with the measurement that
killed it, and they are all in chapter 19 — including one where I attributed a
velocity spike to my own guard, wrote it down, and had to retract it the next
day when the measurement did not reproduce.

That chapter is the honest core of the project. A result is only as
trustworthy as the number of chances it had to be wrong.

## How this book is organised

- **Part 0 (chapters 2–5)** builds the background from nothing: frames and
  poses, how a camera turns the world into numbers, what a matrix inverse
  really does, and what feedback control is.
- **Part 1 (6–9)** derives visual servoing itself, including the interaction
  matrix, line by line.
- **Part 2 (10–13)** is the four failures, one chapter each.
- **Part 3 (14–17)** is the fixes: the 2001 partition, truncation, the runtime
  switch, and the guard that shipped.
- **Part 4 (18–20)** is how any of it was made trustworthy: finite
  differences, the test suite, the dead claims, and a rendered camera with a
  real detector.
- **Part 5 (21–26)** is the robot: ROS 2 from zero, the node, the Gazebo cell,
  kinematics, and the two findings that only appeared on an arm.
- **Part 6 (27–29)** is the limits, how to explain the work out loud, and a
  glossary.

Every chapter opens in plain language and ends with a spoken version you could
say in an interview. Numbers are quoted exactly as the repository reports
them, because a number you cannot reproduce is an opinion.

## Say it like this

> A camera-guided robot judges distance by how big the marker pattern looks.
> When the part folds or tilts, that pattern shrinks for reasons that have
> nothing to do with distance, and the controller drives at the part. I
> measured the four ways this class of controller goes blind, built a one-line
> runtime check that catches the case that matters, and put it on a simulated
> UR5e in ROS 2 and Gazebo to see what a real arm changes.
