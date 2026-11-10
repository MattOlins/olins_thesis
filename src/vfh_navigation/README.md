# Modular 120-degree VFH navigation

Both `vfh` and `vfh_astar` use `VFHController.step(scan, pose, yaw_rate, target, dt)`.
The latter changes the target to a waypoint on an A* route. The local controller
never uses the map to mark unseen LiDAR directions clear. No DRL is implemented.

## Build and test

Activate the ROS 2 Jazzy environment, then from the workspace root. These commands
stop if the build fails, so tests do not accidentally run against an old install:

```bash
colcon build --symlink-install --packages-up-to thesis_test &&
source install/setup.bash &&
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest \
  src/vfh_navigation/test/test_navigation_core.py \
  src/vfh_navigation/test/test_ros_adapter.py \
  src/vfh_navigation/test/test_packaging.py
```

The plugin-autoload setting avoids an incompatibility between newer pytest and
Jazzy's `launch_testing` plugin. The pure core suite also runs with ordinary Python
and NumPy, without ROS. ROS adapter tests skip when ROS Python modules are absent.
The package installs the active `config/vfh_params.yaml` explicitly; archived
`*_og` configurations and stale build-directory symlinks are not installation inputs.

## Establish the shared baseline

```bash
ros2 launch my_robot_bringup navigation.launch.py \
  scenario:=open_field navigation_mode:=vfh
```

Available scenarios: `open_field`, `single_obstacle`, `corridor`, `u_world`, and
`obstacle_world`. Each manifest defines the world, initial pose, final goal,
timeout, and saved-map prefix. Both modes receive exactly the same local parameter
file, footprint, 120-degree sensor, and speed limits. All supplied worlds use a
real-time factor of 1. The robot spawns at z=0.12 m and settles onto its wheels,
avoiding initial wheel/ground interpenetration.

The radius is 0.33 m (enclosing the URDF's wheels as well as the chassis), with a
0.10 m margin. The LiDAR's 0.111125 m forward mounting offset comes from TF. Scan projection
uses TF at acquisition and odometry timestamps to compensate robot motion before
updating the occupied-only memory.
`stop_distance` is measured from the robot origin, not the LiDAR origin.
`observed_clearance` subtracts the uninflated robot radius from measured hit
distances; it is limited to observed surfaces and is not contact ground truth.

Standalone `ros2 run vfh_navigation vfh_node` retains `(goal_x, goal_y)` in `odom`
by default. Scenario launches explicitly use `map`. Send a `PoseStamped` on
`/goal_pose` or use RViz's goal tool to replace the goal and reset all history.
Scenario launches gate both modes on map availability, without giving the VFH-only
controller map geometry. Parameters are startup settings; restart the node to
change modes or tuning.
Final orientation is not controlled.

## Premap each scenario

For comparisons, prepare a complete route in known free space before trial runs:

```bash
ros2 launch my_robot_bringup navigation.launch.py \
  scenario:=u_world navigation_enabled:=false slam_mode:=mapping
```

Teleoperate on `/cmd_vel` to map the start, destination, and passages around the
U-shaped obstacle. Rotate the physical 120-degree sensor by turning the robot;
do not change the URDF FOV. Navigation is disabled during premapping, so only the
teleoperation controller should publish motion commands. Return to the manifest's
initial pose. Map the entire safe footprint around the start and goal from
multiple viewpoints; a stationary rotation or a single out-and-back line can
leave nearby unknown holes that correctly block footprint inflation. The
`start_unknown`, `goal_unknown`, and `*_insufficient_clearance` diagnostics
distinguish these cases from an unreachable route. Do not clear these cells
artificially to make the planner start. Ensure map coordinates and the scenario's goal agree; Gazebo world
coordinates must not be assumed identical to map coordinates after arbitrary
localization or mapping initialization.

Save the SLAM Toolbox pose graph, using an **absolute path prefix without an
extension**, and an existing writable parent directory:

```bash
mkdir -p "$PWD/maps"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$PWD/maps/u_world'}"
```

The launch removes VS Code's inherited `SNAP_COMMON` from the SLAM process;
otherwise SLAM Toolbox can prepend the editor's Snap directory to absolute paths.

Keep both `u_world.posegraph` and `u_world.data`. A PGM/YAML occupancy map alone
cannot initialize SLAM Toolbox localization. Stop the premapping launch before
starting a trial. Repeat for each scenario.

```bash
ros2 launch my_robot_bringup navigation.launch.py \
  scenario:=u_world navigation_mode:=vfh_astar slam_mode:=localization \
  saved_pose_graph:="$PWD/maps/u_world" initial_pose:='[0.0, 0.0, 0.0]'
```

Use the same localization launch and saved graph for the `vfh` comparison.
For nonzero map/world initial-pose offsets, use a scenario whose initial pose and
goal are aligned with the saved map. The current supplied scenarios start at zero.
For a GUI-free run add `headless:=true rviz:=false`; this enables headless rendering
for the GPU LiDAR as well as running only the Gazebo server.

## Inspect planning and failures

RViz displays `/map`, the 120-degree `/scan`, TF, `/global_path`, and `/local_goal`.
The required chain is `map -> odom -> base_footprint -> base_link -> lidar`.
SLAM supplies `map -> odom`; the existing Gazebo ground-truth odometry source
supplies `odom -> base_footprint` in both modes. Switching to wheel odometry is a
separate experimental change.

`/navigation_state` is a `diagnostic_msgs/DiagnosticArray` published at 10 Hz with
state/reason, mode, remaining distance, progress, selected steering, observed
clearance, replan count, stuck retries, and turn reversals. `/local_goal` is in
`odom`; `/global_path` is in `map` and is cleared explicitly when invalidated.

Only occupancy values 0..25 are traversable. Unknown, uncertain, occupied, and
outside-map space are blocked. Inflation includes occupied-cell extents and continuous swept-circle checks, so
marginally narrow corridors can be rejected. A* uses eight neighbors with no
corner cutting. Waypoint lookahead is 0.60 m, with checked connections around
corners. Invalid or unreachable endpoints are reported, never silently snapped.
An unreachable goal waits for a changed map or replacement goal; this stack does
not explore unknown space automatically.

The shared goal weight is 3.0, exceeding heading plus smoothness weights (1.0
each); the previous 2:1:1 combination could produce flat steering costs that
preserved the wrong heading near an off-axis goal. Stuck detection also counts
bounded improvement in target alignment during normal turning, so a legitimate
turn toward a rear goal is not mistaken for a deadlock. Worsening alignment does
not subtract from real translation progress. Net yaw improvement can receive
bounded turn grace (the five-second window plus a half-turn at the configured
maximum angular speed); repeated oscillations and indefinite spinning still
trigger a stuck state. Reported distance progress remains positional/route progress.
The avoidance-side penalty applies only to an observed obstacle blocking the
goal, avoiding a permanent commitment away from a goal outside the scan. Forward
motion additionally requires the current heading, as well as the selected sector,
to be clear; the robot turns first when its selected safe direction is off-axis.
Occupied hits are retained in an odometry-frame voxel cache for five seconds of
valid local control (0.05 m cells). The cache can only add obstacle constraints; it
never authorizes an unobserved direction. It protects side clearance when a close
obstacle leaves the partial FOV. This safety cache survives goal replacement, while
steering/route/progress histories reset; episode and clock resets clear it. Stale
inputs suspend control and conservatively retain cached obstacles. Moving-object
remnants can therefore delay movement until the occupied-only memory expires.

Route invalidation, deviation, and stalled progress cause replanning, throttled
to once per simulation second. Three unsuccessful/unchanged stuck-triggered
replans latch `FAILED`; a new goal or episode reset clears it. VFH alone stops
and reports its stuck state. There is no automatic reversing or special blind
recovery spin. Transient local obstacles remain subject to LiDAR avoidance and
stopping; a saved static map does not predict moving obstacles.

## Record paired trials

After premapping all requested scenarios:

```bash
ros2 run thesis_test compare_navigation \
  --scenarios open_field single_obstacle corridor u_world \
  --maps-dir "$PWD" --output "$PWD/results/run_01" --trials 3
```

The manifest's default `maps/u_world` is resolved relative to `--maps-dir`.
The runner alternates mode order, starts fresh Gazebo/SLAM/navigation processes
for each episode, records rosbag topics before launch, checks sensor/map/TF
readiness, and stops on success, failure, or simulation timeout. A wall-clock
watchdog also catches a stalled clock. Existing trial directories are not
silently overwritten. Only processes launched by the runner are terminated.
Do not run another simulator or velocity publisher in the same ROS/Gazebo domain.

Outputs include per-episode logs, rosbag data, configuration/scenario copies,
artifact hashes, trajectory and navigation CSVs, summary JSONs, and aggregate
`summary.csv`. Time includes planning from the first active navigation state;
trajectory length excludes pre-navigation settling. Failures are retained.
Collision annotations remain null until independently annotated. Do not derive a
collision rate from minimum observed scan clearance.

Validate open space, a single obstacle, and the corridor before the U-world.
For each mode confirm actual commands, TF, stop behavior on missing/stale input,
goal replacement, and episode resets on the thesis machine. Offline A* success
alone does not establish that the physical simulated robot follows the route.

The historical `vfh_node_og.py` remains unchanged and is not selectable as an
operational baseline: it references uninitialized `previous_direction`. No claim
of an original-controller comparison is made by the default paired runner.

See [VALIDATION.md](VALIDATION.md) for executed tests, live outcomes, and the
remaining U-world SLAM-map limitation.
