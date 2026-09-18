# 11. Building the arm environment: the full ROS 2 stack

## In plain words

The numbers so far came from a camera flying freely through space. A real
robot cannot do that: it has joints with limits, a controller that takes
position commands, and a camera bolted to its wrist. So the last phase put the
whole thing on a simulated **UR5e** industrial arm in **Gazebo**, driven
through **ROS 2**, with the guard deciding the control law every step.

This chapter is the build: what runs, how the pieces talk, and every problem
that had to be solved to make them talk.

## What is actually running

Two robot cells in **one** Gazebo world, so they share a physics clock and are
frame-locked by construction. Two separate simulators would drift apart and
the comparison would be worthless.

```
                    ┌──────────── Gazebo (gz sim) ─────────────┐
                    │  physics, two UR5e arms, two panels,     │
                    │  wrist cameras, film cameras             │
                    └───┬───────────────┬──────────────────┬───┘
       gz topics        │               │                  │
                 ros_gz_bridge   gz_ros2_control      ros_gz_bridge
                        │               │                  │
        /left/camera/image      /left/controller_manager   /clock
        /right/camera/image     (JointTrajectoryController) /fold_left
                        │               │                  │
                    ┌───┴───────────────┴──────────────────┴───┐
                    │  fold_node (rclpy, use_sim_time)         │
                    │  detect → guard → control law → Jacobian │
                    │  → JointTrajectory per cell              │
                    └──────────────────────────────────────────┘
```

Packages: `blindspot_ros` (the guard node and the RViz simulation, chapter 10)
and `blindspot_arm` (everything on this page).

## The robot description

`ur5e_camera.urdf.xacro` wraps the standard `ur_description` macro and adds:

- a **camera link** on `tool0` with a Gazebo camera sensor: 640×480, 60°
  horizontal field of view, 20 Hz, publishing on a per-side topic;
- an **optical frame** child link using the computer-vision convention (+z
  forward, +y down), which is the frame the guard's normalised coordinates
  live in;
- a `ros2_control` block whose hardware plugin is
  `gz_ros2_control/GazeboSimSystem`, declaring a **position** command
  interface and **position, velocity, effort** state interfaces per joint;
- the `gz_ros2_control` system plugin, given `<ros><namespace>` so the two
  cells get separate controller managers.

It takes `prefix` and `ns` arguments, so the same file spawns
`left_shoulder_pan_joint` under `/left` and `right_…` under `/right`. One
description, two cells, no copy-paste.

## The controllers

`config/controllers_left.yaml` (and its right twin):

```yaml
/left/controller_manager:
  ros__parameters:
    update_rate: 200
    joint_state_broadcaster: {type: joint_state_broadcaster/JointStateBroadcaster}
    arm_controller:          {type: joint_trajectory_controller/JointTrajectoryController}

/left/arm_controller:
  ros__parameters:
    joints: [left_shoulder_pan_joint, …]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    allow_nonzero_velocity_at_trajectory_end: true
```

`joint_state_broadcaster` publishes `/left/joint_states`;
`JointTrajectoryController` accepts `trajectory_msgs/JointTrajectory` on
`/left/arm_controller/joint_trajectory`. The servo loop sends a **single
trajectory point** each cycle — the next joint configuration, to be reached in
0.16 s — which is a simple and honest way to drive a position-controlled
industrial arm at 10 Hz.

## The launch file

`fold.launch.py` starts, in order:

1. `gz sim` with the world, `-z <rate>` to set physics steps per wall second;
2. a `ros_gz_bridge` for `/clock` (so everything can run on sim time), for the
   fold commands (ROS `std_msgs/Float64` → `gz.msgs.Double`), and for the film
   cameras (gz images → `sensor_msgs/Image`);
3. per cell: `robot_state_publisher` in its namespace with `use_sim_time`, a
   `ros_gz_sim create` that spawns the robot from its `/robot_description`
   topic, a camera bridge with remappings to `/left/camera/image`, and two
   `controller_manager` spawners.

The spawners are staged with `TimerAction` (6 s, 10 s, 13 s) and given
`--controller-manager-timeout 120`, because the controller manager only exists
after the model is spawned and the plugin has loaded. That ordering is one of
the things that cost a day; see the problem list below.

## Closing the loop: `fold_node`

One rclpy node, one 10 Hz timer, two `Cell` objects. Per cell, per tick:

1. take the latest wrist camera frame,
2. **detect** the six dark dots (`detect.py`: threshold, connected components,
   keep the six most *similar*-area blobs),
3. order them by angle around their centroid and convert to normalised
   coordinates with the intrinsics,
4. ask the **guard** (`blindspot.FeatureGuard`, calibrated at startup on the
   known-good hexagon) for a reading,
5. pick the control law — the left cell always runs the partition, the right
   cell obeys the guard,
6. compute the camera twist with the **same law the numpy study uses**
   (`law.py`, numpy only, imported by `tests.py` so the test suite checks the
   law the arm actually runs),
7. map the twist to joint velocities through the arm's **geometric Jacobian**,
   clip to ±1.5 rad/s, integrate one step, and publish the next joint
   configuration,
8. apply the shared **protective stop**: if the *next* pose would bring the
   camera within 12 cm of the part, hold position and latch, the way a real
   cell would until an operator resets it,
9. log everything — sim time, fold angle, feature count, signal, threshold,
   margin, decision, commanded speed, standoff, joint angles — to CSV.

## Kinematics without a kinematics library

`kin.py` parses the URDF with `urdf_parser_py` and builds forward kinematics
and the geometric Jacobian in pure numpy: about 90 lines, no KDL, no MoveIt.

The reason it is trustworthy is not that it is short: it is
`verify_against_tf()`, which compares this FK against what
`robot_state_publisher` reports over TF. **Agreement to 1e-6 m.** A wrong
axis or a missed fixed joint shows up immediately, which is exactly how you
want a hand-rolled kinematics file to be checked.

## Sim time, and why it matters here

Every node runs with `use_sim_time`, driven by `/clock` from Gazebo. The film
cameras render 1080p at 60 Hz, which makes the simulation run at roughly a
quarter of real time — and that must not change the control loop. On sim time
it doesn't: only wall-clock duration changes.

This was a real bug first. The original node used a wall-clock timer, so
turning on recording silently changed the controller's effective rate.

## The problems, in the order they cost time

**Environment and launch**

- A shell that has sourced a conda ROS environment feeds conda libraries to
  system binaries and breaks them in confusing ways. Both run scripts start by
  clearing `LD_LIBRARY_PATH`, `PYTHONPATH`, `CMAKE_PREFIX_PATH` and
  `AMENT_PREFIX_PATH`.
- Gazebo could not find the world's models or the `gz_ros2_control` plugin
  until `GZ_SIM_RESOURCE_PATH` and `GZ_SIM_SYSTEM_PLUGIN_PATH` were set in the
  launch environment.
- `robot_description` has to be passed as `ParameterValue(..., value_type=str)`
  or the launch system infers the wrong type from the XML.
- Controller spawners raced the controller manager: fixed with staged
  `TimerAction`s and a 120 s timeout rather than a sleep-and-hope.

**The world**

- The panel model cannot be `<static>` because its joints must move, so
  gravity dropped it from z = 0.359 to 0.085, out of the camera's view. Fixed
  with an explicit fixed joint anchoring the post to `world`.
- Prefixing the robot's `world` link broke that anchoring, because the joint
  then referred to a link that no longer existed under that name.

**Perception**

- `camera_info` never arrived on the ROS side, and waiting for it stalled the
  servo loop so both arms sat still while the HUD read 0/6 features. The node
  now takes intrinsics from the sensor definition in the URDF and says so in a
  comment, with the `(W−1)/2` principal point convention from chapter 9.
- The detector sometimes found **seven** blobs: at close standoff the robot's
  own dark wrist enters the frame. Taking the six *largest* blobs let the
  wrist masquerade as a marker. Taking the six most **similar in area** fixed
  it, because the six real markers are identical circles.
- Ordering six identical dots by angle fixes their order but not where the
  order *starts*, so a rotated hexagon can pair every dot with the wrong
  target. Measured: one cell converged to 0.090 while its twin diverged to
  0.530 running identical code. Fixed by choosing the cyclic shift with the
  smallest residual.
- Joint states arrive in alphabetical order, not URDF order. Reading them
  positionally gives a plausible, wrong arm.

**Process hygiene**

- `pkill -f <pattern>` matches the running shell's **own** command line, so a
  cleanup line kept killing the session. Bracketed patterns (`[d]uel_node`)
  fix it.
- Gazebo's process is not called `gz sim`; on this version it is
  `gz-sim-main`. Stale servers from earlier runs stacked up and silently
  corrupted measurements until every run was isolated on its own
  `ROS_DOMAIN_ID` and the cleanup was made reliable.
- A background job in a non-interactive script **ignores SIGINT**, so the
  first cleanup handler did nothing; and bash defers a trap until the
  *foreground* command returns, so a node that never exits made TERM a no-op.
  The run script now starts each child with `setsid` and kills process groups.

None of these are control theory. All of them are what "I ran it on a robot"
actually costs, and being able to tell that story is half of what a robotics
interview is asking about.

## Say it like this

> The last phase put the whole thing on ROS 2: two UR5e cells in one Gazebo
> world, gz_ros2_control with a JointTrajectoryController per arm, a
> ros_gz_bridge for clock, commands and images, and one rclpy node closing
> perception → guard → control law → the arm's own Jacobian at 10 Hz on
> simulation time. The kinematics are hand-rolled numpy checked against TF to
> a micrometre, and the control law the arms run is the same module the test
> suite imports, so the tests check the law the robot actually executes rather
> than a copy of it.
