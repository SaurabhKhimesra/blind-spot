# 22. Packaging it: the library and the ROS 2 node

## In plain words

A result in a script is not something anyone can use. Two things had to exist
before this could be called finished: a small library someone can import into
their own control loop, and a **ROS 2 node** that publishes the decision so a
real robot stack can consume it.

ROS 2 is the standard plumbing for robots: independent programs called
**nodes** talk to each other by publishing and subscribing to named
**topics**, with typed **messages**. The guard is a node. It reads detections
and camera calibration, and it publishes one boolean.

## The library contract

```python
from blindspot import FeatureGuard
guard = FeatureGuard.load("my_target.json")

if guard.partition_ok(s_visible):        # normalised image coordinates
    v = my_partitioned_control(...)
else:
    v = my_plain_control(...)

h = guard.evaluate(s_visible)   # signal, threshold, margin, n_features, decision
```

Four design rules, all of them refusals:

1. **No default threshold.** `FeatureGuard(None)` raises, with the calibrate
   command in the error text and the reason: the threshold carries target
   scale and working distance (measured 3.9e-3 for one target, 1.7e-2 for
   another).
2. **No unit guessing.** The guard works in normalised image coordinates.
   `blindspot/units.py` converts from pixels and *requires* the principal
   point; it will not assume `W/2` (chapter 20 explains why that half pixel
   matters).
3. **No controllers in the product.** The verified control laws live in
   `blindspot/reference/`, whose docstring says they are "EXAMPLES, not the
   product". The shipped thing decides; it does not drive.
4. **No hidden state.** `evaluate()` returns the signal, the threshold, the
   margin, the feature count and the decision, so a log line explains itself.

`test_blindspot.py` has 34 checks, including checks that these refusals still
refuse.

## The ROS 2 node

Package `blindspot_ros`, node `guard_node`:

| direction | topic | type |
|---|---|---|
| in | `detections` | `vision_msgs/Detection2DArray` |
| in | `camera_info` | `sensor_msgs/CameraInfo` |
| out | `~/partition_ok` | `std_msgs/Bool` |
| out | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |

```bash
ros2 run blindspot_ros guard_node --ros-args \
    -p calibration:=/abs/path/my_target.json \
    -r detections:=/aruco/detections \
    -r camera_info:=/camera/camera_info
```

It does **not** control anything. Keep your servo loop, gate it on the Bool.
That is a deliberate boundary: a node that both decides and drives is a node
nobody can adopt incrementally.

Three things the node handles that are easy to get wrong in a real stack:

**Ordering.** The guard signal is a shoelace polygon area, so it depends on
the order of the points — and a detector reports whatever order it happens to
find. An order that crosses itself gives a small area and fires the guard for
a reason that has nothing to do with geometry. So detections are ordered **by
marker id**, which reproduces the order calibration used; without ids there is
an angular fallback; and the ordering actually used is published in the
diagnostic. The node also cross-checks the shoelace area against the convex
hull area and raises the diagnostic to **ERROR** when they disagree, because
at that point the decision is not measuring what it claims to.

**Intrinsics.** From `camera_info` when the camera is calibrated. If it
publishes zeros, set `fovy_deg` and the node falls back to a principal point
of `(W−1)/2` — and says in the diagnostic which path it used.

**Refusing to start.** `calibration` is a required parameter. Without it the
node exits immediately and prints the calibrate command. Same reason as the
library: a default threshold would be a silent wrong answer.

All the ordering, id and unit logic lives in `blindspot/ros_bridge.py`, which
imports **no ROS at all**, so it is unit-tested on a machine with no ROS
installed. The rclpy node is a thin wrapper over it.

## Verified running

`demo.launch.py` starts the guard plus a `fake_detections` node that needs no
camera and deliberately **shuffles** the detection order every frame, so the
ordering path is exercised rather than assumed:

| phase | n | margin | decision | diagnostic | ordering |
|---|---|---|---|---|---|
| healthy, 6 markers | 6 | 1.564 | `partition_ok` | OK | id |
| occluded, 2 markers | 2 | 0.000 | `drop_partition` | WARN | id |
| collapsed target, 6 markers | 6 | 0.221 | `drop_partition` | WARN | id |

Those margins reproduce the library's own numbers (1.55 and 0.22 in
`examples/quickstart.py`), so the ROS path is not computing something
different. The third row is the case feature counting cannot see: six markers
present, and the guard correctly drops the partition anyway.

## The live simulation

`sim.launch.py` opens RViz and runs a servo loop **closed through the guard
node**: the control law is chosen every step by the Bool on
`/blindspot_guard/partition_ok`.

```
partition_ok == True   -> the 2001 partitioned law
partition_ok == False  -> plain truncated law on the full interaction matrix
```

`sim_node` publishes `camera/image`, `markers` (a `MarkerArray` showing the
target and the camera frustum in 3D), `tf`, `detections` and `camera_info`,
and cycles healthy → occluded to two markers → collapsed target, re-posing the
camera at each change so every regime shows a real approach rather than a
converged still. The images are drawn with numpy, so there is no OpenCV or
`cv_bridge` dependency. `rviz:=false` runs it headless.

## A bug that only the real distro found

Verified on **ROS 2 Lyrical** (Ubuntu 26.04, system Python 3.14, numpy 2.3.5)
and also on Jazzy via RoboStack. Running it on the real distro immediately
found a bug Jazzy did not: `rclpy` on Lyrical shuts the context down inside
its own SIGINT handler, so `spin()` raises `ExternalShutdownException` rather
than `KeyboardInterrupt`. Catching only the latter exits with status 1 on a
clean Ctrl-C. All three nodes now catch both.

Being distro-agnostic in principle was an argument. The argument was wrong,
and one apt install settled it.

## Say it like this

> The deliverable is a small library plus a ROS 2 node that publishes one
> boolean on a topic, so it drops into someone else's servo loop without
> taking it over. The node handles the things that actually bite in a real
> stack: detection ordering, because the signal is a polygon area and a
> detector returns points in any order; intrinsics from camera_info with a
> stated fallback; and refusing to start without a calibration file, because a
> default threshold would be a silent wrong answer.
