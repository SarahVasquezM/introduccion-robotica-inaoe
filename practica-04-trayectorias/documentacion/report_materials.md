# Practice 4: report materials

## Objective and architecture

Navigate a simulated simple_drone between arbitrary positions using a
collision-free XY route. The practica4_rrt node exposes the existing
`control_action/action/PIDcontrol` interface at `/pidcontrol`. Each accepted goal
uses the current pose as A and the requested `(xd, yd, zd, wd)` as B, then plans
and executes a complete continuous trajectory during `tf`. A new goal can be sent
after completion without restarting the simulator.

World geometry and occupancy come from world_config.yaml. Obstacles are static
5 m high boxes. The occupancy resolution is 0.1 m. Obstacles and map boundaries
are inflated by drone radius 0.4 m plus margin 0.50 m. Cells touching an inflated
box are occupied. Every candidate edge is intersected with occupied cell boxes
using slab intervals, so thin crossings and tangencies are rejected.

## Algorithm and equations

RRT uses seeded random free-space sampling, goal bias 0.15, nearest-neighbour
search by Euclidean distance, steering by at most 1.0 m, collision checks on the
whole edge, and parent pointers for path reconstruction. A goal connection is
attempted within 0.8 m and must pass the same collision check. The maximum
iteration count is 10,000. Exhaustion produces a controlled planning error;
it does not establish that no mathematical solution exists.

Smoothing first performs up to 200 seeded line-of-sight shortcuts. The remaining
corners are replaced by quadratic Bezier arcs whose samples and connecting edges
are checked against the same inflated occupancy map. The dense collision-free
curve is parameterized by arc length. A single global cubic progress function
`s(u)=3u²-2u³`, `u=clamp(t/tf,0,1)`, moves along the entire curve. It has zero
velocity only at A and B; velocity remains continuous and nonzero through the
intermediate RRT points.

Tracking combines curve velocity feed-forward with position feedback. XY commands
are rotated into the drone body frame; altitude and yaw use their corresponding
references. Intermediate errors do not end or pause the action. At `tf`, velocity
is set to zero and the final pose determines `SUCCEEDED` or `ABORTED`.

The report must explain the practical time-selection rule. The three-dimensional
route length includes the rounded XY curve and the altitude change. Execution is
rejected before motion when `tf < 2L`, and the result prints the required minimum.
No settling duration is appended at intermediate points or after `tf`. The result
reports desired time, measured wall time, route length, and final x/y/z/yaw errors.
The default final limits are 0.03 m position norm and 0.05 rad yaw.

Gazebo contacts, excessive roll/pitch, stale pose data, and entry into the physical
safety envelope abort the goal and command zero velocity. A separate larger margin
is retained for planning so tracking error does not imply physical collision.

## Measured planning results (no flight implied)

These rows were obtained from the real offline planner with the committed config.
Refer to results/practica4/offline/*/results.json and rrt_map.png.

| Scenario | A → B (m) | Seed | Iterations | Raw length (m) | Smoothed length (m) | Nominal T (s) | Flight wall time (s) | Tracking RMSE (m) |
|---|---|---:|---:|---:|---:|---:|---|---|
| simple | (0,0) → (0,-2) | 11 | 16 | 3.019 | 2.000 | 12 | To measure | To measure |
| detour | (0,0) → (6,0) | 22 | 275 | 13.580 | 11.669 | 60 | To measure | To measure |
| alternate | (0,0) → (-6,4) | 33 | 82 | 9.615 | 8.900 | 45 | To measure | To measure |

Do not fill flight metrics from offline planning. Distinguish requested nominal
time, measured simulation duration and measured wall-clock duration. Use flight.csv
only for an executed run; mark failed or canceled runs explicitly. Feedback and
pose are asynchronous, so account for timestamp alignment when reporting errors.

## Evidence checklist and video script

Capture the Gazebo obstacle world, RViz occupancy (including inflation), A/B,
raw and smoothed paths, full tree PNG, action result and measured flight trace.
Save launch commands, seed, parameters, JSON and CSV together. Record actual
collisions or failed segments; do not present planned collision freedom as proof
that the physical simulated trajectory never collided.

For each video: introduce scenario A/B, altitude, seed and nominal time; show the
world and occupied map; show the tree and compare routes; demonstrate takeoff;
send the destination with `/pidcontrol`; follow the continuous execution; finish
with the action result and desired versus real duration. Then send a second goal
without restarting to demonstrate replanning from the current pose.

Optional rosbag in terminal 3 (source ROS and workspace first):

```bash
ros2 bag record -o results/practica4/bag_detour /clock /simple_drone/gt_pose /simple_drone/cmd_vel /rrt_map /rrt/path_raw /rrt/path_smoothed
```

Start recording before launch if you need the first latched map/path publications,
or configure transient-local subscription overrides. Save results.json alongside
the bag; action execution evidence also resides in flight.csv and the action result.
Stop recording with Ctrl+C after the flight result. Record screen video separately;
no screen recording or flight result is generated by the offline scripts.
