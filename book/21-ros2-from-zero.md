# 21. ROS 2, from zero

Everything from here on runs on ROS 2, so this chapter explains it from
nothing. If you already write ROS nodes, skip to the last section, which is
what this project specifically uses.

## The problem ROS solves

A robot is many programs at once: one reads the camera, one finds markers, one
decides what to do, one talks to the motors, one logs everything. They start
and stop at different times, some crash, some run on a different machine.

Wiring them together by hand — sockets, file formats, start-up order — is a
job nobody wants to do twice. **ROS 2** (Robot Operating System 2) is the
standard plumbing that does it for you. It is not an operating system; it is a
set of libraries and conventions.

## The five ideas

**Node.** One program that does one job. `guard_node` is a node.

**Topic.** A named channel. Nodes **publish** to it, other nodes
**subscribe**. Nobody needs to know who else is connected, so you can add a
logger or a visualiser without touching anything.

**Message.** The typed shape of what travels on a topic.
`sensor_msgs/Image` is a picture, `std_msgs/Bool` is a true/false,
`vision_msgs/Detection2DArray` is a list of detections. Types are shared
across the whole ecosystem, which is what makes independently written nodes
interoperate.

**Parameter.** A named setting a node reads at start-up — a file path, a
threshold, a rate.

**Launch file.** A Python script describing which nodes to start, with which
parameters and remappings, and in what order. One command brings up the whole
system.

Two more that this project needs:

**TF.** The transform tree: a live record of where every frame is relative to
every other (chapter 2's poses, published continuously). Ask "where is the
camera in the robot's base frame right now?" and TF answers.

**Simulation time.** When a simulator is running, it publishes a clock on
`/clock`, and nodes started with `use_sim_time` follow *that* clock instead of
the wall clock. It matters more than it sounds: if the simulator runs at a
quarter of real speed because it is rendering 1080p video, a node on wall-clock
time silently runs its control loop four times too often per simulated second.
That was a real bug in this project (chapter 23).

## Names, namespaces and remapping

Topics are strings, so two copies of the same node would collide. **Namespaces**
prefix them: the same code launched under `/left` and `/right` publishes
`/left/joint_states` and `/right/joint_states`. **Remapping** rewires a
topic name at launch time without editing code, which is how the guard node
listens to your detector's topic rather than a hard-coded one:

```bash
ros2 run blindspot_ros guard_node --ros-args \
    -p calibration:=/abs/path/my_target.json \
    -r detections:=/aruco/detections
```

## ros2_control, in one page

Motors need commands at a steady, high rate — hundreds of hertz — with a
consistent interface across different hardware. `ros2_control` is the
framework for that.

- A **hardware interface** exposes a robot's joints as **command interfaces**
  (what you can set: position, velocity, effort) and **state interfaces**
  (what you can read).
- A **controller** turns a higher-level goal into those commands. This project
  uses `JointTrajectoryController`, which accepts a list of joint positions
  with times and interpolates between them, and `JointStateBroadcaster`, which
  publishes the measured joint positions.
- The **controller manager** loads, starts and stops controllers, and runs
  them in a fixed-rate loop (200 Hz here).

In simulation, the hardware interface is provided by `gz_ros2_control`, which
plugs the same machinery into Gazebo. That is the property worth having: the
control side of the stack does not know it is in a simulator.

## Gazebo and the bridge

**Gazebo** simulates physics and renders sensors. It has its own message
system, so `ros_gz_bridge` translates between the two worlds in whichever
direction you specify:

```
/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock          # gz -> ROS
/fold_left@std_msgs/msg/Float64]gz.msgs.Double        # ROS -> gz
```

The bracket points the way the data flows. Anything not bridged simply does
not exist on the ROS side, which is one of the ways an afternoon disappears.

## What this project uses

| piece | where |
|---|---|
| nodes, topics, parameters, namespaces, remapping | everywhere |
| `vision_msgs`, `sensor_msgs`, `std_msgs`, `diagnostic_msgs` | the guard node (chapter 22) |
| `trajectory_msgs`, `visualization_msgs` | the arm and the RViz simulation |
| launch files with staged actions | both demos |
| TF | verifying hand-written kinematics (chapter 24) |
| RViz | the live guard simulation |
| `ros2_control` + `gz_ros2_control` + `JointTrajectoryController` | the arm cells (chapter 23) |
| `ros_gz_bridge`, `ros_gz_sim` | clock, fold commands, camera images, spawning |
| simulation time | every node in the arm demo |

Two packages hold it: `blindspot_ros` (the guard node, a fake-detection
publisher, and an RViz simulation) and `blindspot_arm` (the Gazebo cells, the
control law, the recorder).

## Say it like this

> ROS 2 is the plumbing: independent programs called nodes exchange typed
> messages over named topics, started together by a launch file, with
> parameters and remapping so the same code runs in two namespaces. On top of
> that, ros2_control owns the joint-level loop and gz_ros2_control gives the
> same interface in Gazebo, so the control side never knows it's in a
> simulator. And everything runs on the simulator's clock, not the wall clock.
