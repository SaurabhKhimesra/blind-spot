# 23. Building the Gazebo cell

## In plain words

Everything so far assumed a camera that could fly anywhere. A real robot
cannot: it has joints with limits, a controller that takes position commands,
and a camera bolted to its wrist.

So the last phase put the whole thing on a simulated **UR5e** industrial arm
in Gazebo, driven through ROS 2, with the guard choosing the control law every
step. And to make the comparison airtight, **two** arms in **one** world: the
left one always runs the 2001 partition, the right one obeys the guard.

One world means one physics clock, so the two cells are frame-locked by
construction. Two separate simulators would drift apart and the comparison
would be worthless.

## What is running

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
                    │  detect -> guard -> control law ->       │
                    │  Jacobian -> JointTrajectory per cell    │
                    └──────────────────────────────────────────┘
```

## The robot description

`ur5e_camera.urdf.xacro` wraps the standard `ur_description` macro and adds:

- a **camera link** on `tool0` carrying a Gazebo camera sensor: 640×480, 60°
  horizontal field of view, 20 Hz;
- an **optical frame** child link in the computer-vision convention (+z
  forward, +y down) — the frame the guard's normalised coordinates live in;
- a `ros2_control` block whose hardware plugin is
  `gz_ros2_control/GazeboSimSystem`, declaring a **position** command
  interface and **position, velocity, effort** state interfaces per joint;
- the `gz_ros2_control` system plugin with `<ros><namespace>`, so the two
  cells get separate controller managers.

`xacro` is a macro language for URDF: the file takes `prefix` and `ns`
arguments, so the same description spawns `left_shoulder_pan_joint` under
`/left` and `right_…` under `/right`. One file, two cells, no copy-paste and
no chance of them differing.

## The controllers

```yaml
/left/controller_manager:
  ros__parameters:
    update_rate: 200
    joint_state_broadcaster: {type: joint_state_broadcaster/JointStateBroadcaster}
    arm_controller:          {type: joint_trajectory_controller/JointTrajectoryController}

/left/arm_controller:
  ros__parameters:
    joints: [left_shoulder_pan_joint, ...]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    allow_nonzero_velocity_at_trajectory_end: true
```

The servo loop sends a **single trajectory point** each cycle: the next joint
configuration, to be reached in 0.16 s. That is a simple, honest way to drive
a position-controlled industrial arm from a 10 Hz vision loop, and it is close
to what a real cell does.

## The world

`panel_fold.sdf` holds, per cell: a post anchored to the world, a light
backdrop, and two flaps on hinges carrying the markers, driven by joint
position controllers on a ROS topic. Plus two **film cameras** — static camera
sensors used only for recording the clip, never by the controller.

Design choices that each fixed a specific problem are in chapter 26, because
they belong to the folding scenario.

## The launch file

`fold.launch.py` starts, in order:

1. `gz sim` with the world, and `-z <rate>` to set physics steps per wall
   second (this is how recording runs are slowed down);
2. a `ros_gz_bridge` for `/clock`, for the fold commands
   (`std_msgs/Float64` → `gz.msgs.Double`), and for the film cameras;
3. per cell: `robot_state_publisher` in its namespace with `use_sim_time`,
   a `ros_gz_sim create` that spawns the robot from its `/robot_description`
   topic, a camera bridge with remappings to `/left/camera/image`, and two
   `controller_manager` spawners.

The spawners are staged with `TimerAction` at 6 s, 10 s and 13 s, with
`--controller-manager-timeout 120`, because the controller manager does not
exist until the model is spawned and the plugin has loaded.

## Simulation time, and the bug that proved it matters

Every node runs with `use_sim_time`. The film cameras render 1080p at 60 Hz,
which drops the simulation to roughly a quarter of real time — and that must
not change the control loop. On sim time it does not: only wall-clock duration
changes.

The first version used a wall-clock timer, so turning recording on silently
changed the controller's effective rate. Nothing crashed. The run just meant
something different from the run before it, which is the worst kind of bug in
a measurement project.

## The problems, in the order they cost time

**Environment**

- A shell that has sourced a conda ROS environment feeds conda libraries to
  system binaries and breaks them confusingly. Both run scripts start by
  clearing `LD_LIBRARY_PATH`, `PYTHONPATH`, `CMAKE_PREFIX_PATH` and
  `AMENT_PREFIX_PATH`.
- Gazebo could not find the world's models or the `gz_ros2_control` plugin
  until `GZ_SIM_RESOURCE_PATH` and `GZ_SIM_SYSTEM_PLUGIN_PATH` were set in the
  launch environment.

**Launch and controllers**

- `robot_description` must be passed as `ParameterValue(..., value_type=str)`,
  or the launch system infers the wrong type from the XML.
- Controller spawners raced the controller manager. Fixed with staged
  `TimerAction`s and a long timeout rather than a sleep and hope.

**The world**

- The panel model cannot be `<static>` because its joints must move, so
  gravity dropped it from z = 0.359 m to 0.085 m, out of the camera's view
  entirely. Fixed with an explicit fixed joint anchoring the post to `world`.
- Prefixing the robot's `world` link broke that anchoring, because the joint
  then referred to a link that no longer existed under that name.

**Process hygiene** (the ones that waste whole evenings)

- `pkill -f <pattern>` matches the running shell's **own** command line, so a
  cleanup line kept killing the session that issued it. Bracketed patterns
  (`[d]uel_node`) fix it.
- Gazebo's process on this version is `gz-sim-main`, not `gz sim`. Stale
  servers from earlier runs stacked up and silently corrupted measurements
  until every run was isolated on its own `ROS_DOMAIN_ID` and the cleanup was
  made reliable.
- A background job in a non-interactive script **ignores SIGINT**, so the
  first cleanup handler did nothing at all. And bash defers a trap until the
  *foreground* command returns, so a node that never exits made SIGTERM a
  no-op too. The run script now starts each child with `setsid` and kills
  process groups.

None of this is control theory. All of it is what "I ran it on a robot"
actually costs, and being able to tell that story is half of what a robotics
interview is asking about.

## Say it like this

> Two UR5e cells in one Gazebo world so they share a physics clock, described
> by one xacro file instantiated twice under different namespaces, with
> gz_ros2_control giving each a controller manager and a
> JointTrajectoryController. The launch file stages the spawners because the
> controller manager doesn't exist until the model is up, everything runs on
> simulation time so recording can slow the simulator without changing the
> control rate, and the panel needed an explicit anchor joint because a model
> with moving joints can't be static and gravity took it out of frame.
