# 5. Feedback control in one chapter

## The loop

Feedback control is three steps, repeated forever:

1. **Measure** where you are.
2. **Compare** with where you want to be. The difference is the **error**.
3. **Act** in a way that reduces the error.

A thermostat does it with temperature. A servo does it with position. This
project does it with the positions of markers in a picture.

The step that matters is the third one: *how much* do you act? Act too weakly
and it takes forever. Act too strongly and you overshoot, come back, overshoot
again — the system oscillates, or goes unstable and never settles.

## Proportional control and exponential decay

The simplest choice is to act in proportion to the error:

```
action = −λ · error
```

`λ` (lambda) is the **gain**. The minus sign says "move against the error".

Ask what this does over time. If the action *is* the rate of change of the
error, then

```
ė = −λ e
```

and that equation has one solution:

```
e(t) = e(0) · exp(−λ t)
```

The error decays exponentially, never quite reaching zero but halving every
`ln 2 / λ` seconds. This is the target behaviour of every IBVS controller
here, and λ = 0.5 unless a chapter says otherwise.

Two honest observations about exponential decay:

- It is **fast when the error is large**. Commanded speed is proportional to
  error, so at the start of a move the controller asks for the most. That is
  exactly when a wrong model hurts most, and it is why "what is the peak
  commanded velocity" is the measure used for the spike failures.
- It **never overshoots** in a simple system. When these controllers do lurch,
  it is never because the gain is too high; it is because the map between
  action and error — the interaction matrix — is wrong or nearly singular.

## Doing it in discrete steps

A computer cannot act continuously. It samples every `dt`, computes, and holds
the command until the next sample. The study uses `dt = 0.033 s` (30 Hz); the
Gazebo arm runs at `dt = 0.1 s` (10 Hz), which is a realistic rate for a
vision loop with a detector in it.

Sampling has a price: for `dt` seconds the robot acts on stale information. A
loop that is stable in continuous time can oscillate when sampled too slowly,
and any delay — camera exposure, detection time, the controller's own
computation — adds to it. The arm chapters keep everything on **simulation
time** for exactly this reason: when recording slowed the simulator to a
quarter of real time, an earlier version used a wall-clock timer and silently
changed the control rate.

## Why a decision, not a gain, is the subject here

The classic knobs of control — gain, filtering, feed-forward — are not what
this project tunes, and it is worth being able to say why.

The failures in part 2 are not "the gain is wrong". They are cases where the
relationship the controller is inverting has stopped being trustworthy: a
direction has gone blind, or a substitute measurement has stopped meaning what
it meant. No value of λ fixes that. Turning the gain down makes the same wrong
motion happen more slowly.

What does fix it is choosing a **different control law** for the duration, and
that is a decision, made every step, on evidence. The evidence is a property
of the geometry, never of the error — because chapter 10 shows the error
reads perfect during the most spectacular failure in the book.

## Say it like this

> Feedback is measure, compare, act. Acting proportionally to the error gives
> exponential decay, which is the behaviour every controller here asks for.
> The failures in this project aren't gain problems — they're cases where the
> map from action to error has gone wrong, and no gain fixes that. So the
> variable I tune is not a gain, it's which control law runs this step.
