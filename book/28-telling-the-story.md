# 28. Telling the story

## The 30-second version

> Image-based visual servoing steers a robot straight from what its camera
> sees. It has a famous failure — asked to rotate 180°, it flies the camera
> backwards — and the standard 2001 fix for that failure causes two worse ones:
> with two features left it spikes nearly 300×, and on a collapsing target it
> can't converge. I made that fix a runtime decision instead of a permanent
> one, with a one-line check on the health of the feature the controller
> relies on. It closes the gap from 11.6× to 0.5×, which killed my own
> hypothesis that the decision needed a learned policy. It ships as a library
> and a ROS 2 node, and runs closed-loop on a UR5e in Gazebo.

## The 2-minute version

Add, in this order:

1. **Why the four failures are different.** Retreat is coupling with a healthy
   matrix; dropout is conditioning; two-features is a structural failure of
   one controller; collapse is missing information that feature counting
   cannot see. One rule has to cover all four, which is why I implemented six
   candidate rules and compared them instead of picking one.
2. **What the guard watches and why.** The 2001 law uses the area of the
   marker polygon as a stand-in for distance, so that area is what has to be
   healthy. The threshold comes from calibration on a known-good part: a few
   thousand healthy viewpoints, first percentile, per target and per number of
   markers visible.
3. **The honest ledger.** The area signal can't tell a degenerate part from a
   healthy one seen at a steep angle. The spectral signal σ₆ can, but it's
   blind to a folding part. Neither dominates, both failure directions are
   measured, and the README says so.
4. **How it was verified.** Derivatives by finite difference, not by reasoning
   about sign conventions — that caught two sign bugs. 177 regression tests,
   including tests that the things that didn't work still don't.
5. **What it runs on.** A ROS 2 node publishing one boolean, and a Gazebo
   UR5e cell where the loop is perception → guard → control law → the arm's
   own Jacobian at 10 Hz on simulation time.

## The 10-minute whiteboard version

Draw these five things in order. Each one is a chapter of this book.

1. **The projection and the interaction matrix** (chapter 7). Point at the two
   rows. Say what each column means. Point out that `vx` and `ωy` look alike
   for a small far target — that's the weak direction everything later turns
   on.
2. **The control law and its inverse** (chapter 8). `v = −λ L⁺ e`. Say: every
   failure in this project happens inside this inverse.
3. **The partition** (chapter 14). Cross out two columns; write `vz` from area
   and `ωz` from line angle beside them. Say: this fixes retreat and makes the
   remaining solve exactly determined, which is the 288× spike.
4. **The switch and the guard** (chapters 16–17). One comparison per step;
   threshold from calibration; what the margin means.
5. **The two counter-examples** (chapters 25–26). Oblique healthy part: area
   fires, σ₆ silent. Folding part: area fires correctly, σ₆ silent at 1.29×.
   Say: same disagreement, opposite verdicts, which is why the repo ships the
   trade rather than a winner.

## Question bank

**"Why not use learning?"**
> I planned to. I set the bar in advance: a hand-written rule had to get the
> two-feature spike under 2× for the learned plan to be dropped. It reached
> 0.5×. Training a policy would have meant data collection, a reward, and an
> uninspectable model, to replace a comparison between two numbers.

**"Why the area and not the matrix spectrum?"**
> Because the area is the quantity the 2001 law actually leans on, and it's
> cheap — no SVD — and steadier under detector noise: 2 px of per-point noise
> moves it 0.33% of its threshold against σ₆'s 1.03%. But I don't claim it
> dominates. On an oblique healthy part it cries wolf, and the cost is a 2 m
> retreat in 100 runs out of 100. On a folding part it's the only one of the
> two that fires. Both are measured and both are in the README.

**"How do you set the threshold?"**
> Calibration on a known-good target: thousands of healthy poses around the
> goal, first percentile, seeded and reproducible, conditioned on how many
> markers are visible because singular-value interlacing means a subset's
> statistic is bounded by the full set's. And it must not be calibrated in
> situ — I measured that: the threshold comes out seven times lower and the
> guard silently never fires again.

**"What happens under noise?"**
> Independent noise it shrugs off, d′ = 454 at realistic levels. The hard case
> is noise correlated with the failure, which is why I built a rendered camera:
> motion blur driven by the camera's own inter-frame motion. The finding is
> that correlated degradation doesn't bias the guard, it removes its input —
> the margin is flat right up to the cliff and then the detector fails
> outright — so a guard failure and a detection failure stay distinguishable.

**"What's the weakest part of this?"**
> Three things. It's simulation, not hardware. The guard watches a quantity
> its own controller regulates, so a slow collapse is masked — measured: 85°
> over 3 s is missed completely. And the shipped truncation constant has a
> constructible case where intervening is worse than doing nothing: τ=1e-3
> stalls at 0.297 m where plain classic IBVS converges exactly.

**"Tell me about something you got wrong."**
> I claimed the guard's switch caused a velocity spike in the arm demo and
> wrote it down. Then I reproduced it across 16 tilt onset/speed combinations
> and the peak velocity was identical under all three rules in 15 of them. The
> spike never reproduced; it was most likely perception noise at a grazing
> view. I corrected it in the file where the wrong claim was rather than
> deleting it.

**"How do you know your implementation is right?"**
> Finite differences. A derivative is a prediction about what happens under a
> small change, so I make the change and compare — 16 checks covering every
> derivative, sign and null space. Two sign bugs in the partitioned law were
> found that way and neither would have been caught by reading the code.

**"What would you do next?"**
> Hardware, and a second signal. The two signals fail in opposite directions,
> so the obvious next step is to run both and report disagreement as its own
> state, rather than picking one. And I'd sweep τ per geometry instead of
> inheriting one constant.

## ROS-specific questions

**"What ROS 2 did you actually use?"**
> Packages, nodes, topics, parameters, namespaces and remappings; launch files
> with staged actions; `vision_msgs`, `sensor_msgs`, `std_msgs`,
> `diagnostic_msgs`, `trajectory_msgs`, `visualization_msgs`; TF; RViz;
> `ros2_control` with `gz_ros2_control` and a `JointTrajectoryController` per
> arm; `ros_gz_bridge` for clock, commands and images; and simulation time
> everywhere.

**"How does the guard node fit into someone else's stack?"**
> It subscribes to `vision_msgs/Detection2DArray` and `sensor_msgs/CameraInfo`
> and publishes `std_msgs/Bool` plus a diagnostic. It doesn't control
> anything, which is deliberate — you keep your servo loop and gate it on the
> Bool, so it can be adopted without rewriting a controller.

**"Why does detection ordering matter?"**
> The signal is a shoelace polygon area, so it depends on point order, and a
> detector returns points in whatever order it found them. A self-crossing
> order gives a small area and fires the guard for a reason unrelated to
> geometry. So detections are ordered by marker id, with an angular fallback,
> the ordering used is published in the diagnostic, and the node cross-checks
> the shoelace area against the convex hull and escalates to ERROR if they
> disagree.

**"How do you handle time in simulation?"**
> Every node runs with `use_sim_time` off `/clock`. It matters here: recording
> 1080p cameras drops Gazebo to about a quarter of real time, and on sim time
> that changes only wall-clock duration. An earlier version used a wall-clock
> timer, so turning recording on silently changed the control rate.

**"How is the arm driven?"**
> The control law produces a camera twist. I map it through the arm's
> geometric Jacobian — hand-written numpy from the URDF, verified against TF
> to 1e-6 m — clip joint velocities to ±1.5 rad/s, integrate one step, and
> publish a single-point `JointTrajectory` to the
> `JointTrajectoryController` with a 0.16 s horizon at 10 Hz.

**"What broke while you were building it?"**
> Pick two from chapter 23. The good pair: the detector finding seven blobs
> because the robot's own wrist enters the frame at close standoff — fixed by
> selecting the six most similar in area rather than the six largest; and
> ordering six identical dots by angle fixing their order but not where the
> order starts, so one cell converged to 0.090 while its twin diverged to
> 0.530 running identical code.

## How to describe the ROS part accurately

Say it this way, because it is both stronger and checkable:

> The control study is numpy, deliberately — it's where the measurements are
> exact and reproducible. The **deployment** is ROS 2: the guard ships as a
> node, and the robot demonstration is a full ROS 2 + Gazebo cell with
> ros2_control, closed loop through the guard.

Do not say the entire project runs on ROS. Anyone who opens the repo sees a
numpy study at its core, and being caught rounding up on a small thing makes
every number you quote suspect. The split is a strength: the study is exactly
reproducible because it is not entangled with a simulator, and the same
control law is then imported by the ROS node so the two cannot drift apart.

## Glossary

| term | meaning |
|---|---|
| **IBVS** | image-based visual servoing: control directly from image error |
| **feature** | a point the detector finds — here, a marker centre |
| **interaction matrix `L`** | maps camera velocity to image-feature velocity |
| **pseudo-inverse** | least-squares "inverse" of a non-square matrix |
| **singular value** | how strongly the matrix responds in one direction |
| **σ₆** | the 6th singular value, padded — zero if the matrix has a null space |
| **truncation (τ)** | skipping directions weaker than τ × the strongest |
| **the partition** | the 2001 scheme driving `vz` and `ωz` from image measurements |
| **the guard** | the runtime check on the partition's own area feature |
| **margin** | signal ÷ threshold; below 1 means the guard fires |
| **node / topic** | a ROS 2 program / a named channel it publishes or subscribes to |
| **ros2_control** | the ROS 2 framework that owns hardware interfaces and controllers |
| **TF** | the ROS transform tree: where every frame is relative to every other |
| **sim time** | clock published by the simulator, so nodes run on its schedule |

## Last thing to say

If the conversation ends and you only get one sentence, use this one:

> The result I'm proudest of is the one that killed my own plan, and the
> repository documents seven of those with the numbers that killed them.
