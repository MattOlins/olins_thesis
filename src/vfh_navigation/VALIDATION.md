# Validation record — November 3, 2026

The modular stack was built and exercised in the installed ROS 2 Jazzy / Gazebo
Harmonic environment. All trials preserved the original 720-ray, 120-degree
LiDAR, 0.33 m robot radius, and 0.10 m safety margin. No original-controller
comparison or DRL improvement is claimed.

## Automated checks

- Navigation: 56 tests passed, including pure-controller/planner tests, actual ROS
  message/TF2 adapter tests, and package lint/docstring checks.
- Bringup: 8 tests passed, including scenario/goal consistency and unchanged FOV.
- Comparison runner: 6 tests passed, including measurements and shutdown cleanup.
- Three existing template copyright checks remain skipped.
- All three packages built successfully using colcon in a temporary install.

Tests include captured pre-refactor histogram/steering outputs, invalid and partial
scans, sensor offset, motion-compensated acquisition timestamps, occupied-only
memory, goal replacement, stale data, clock resets, frame handling, diagonal corner
cutting, continuous footprint clearance, route following, detours, and stale
asynchronous search results. Closed-loop synthetic scans reproduce single-obstacle
avoidance and verify the full 0.43 m clearance from the obstacle surface to the
robot center. This does not establish hardware safety or contact ground truth.

## Live headless runs

| Scenario | Mode / map setup | Outcome | Simulation time | Trajectory |
|---|---|---|---:|---:|
| Open field | VFH / saved SLAM graph localization | Goal reached | 21.1 s active navigation | 3.807 m |
| Open field | VFH + A* / same saved graph and local settings | Goal reached | 21.2 s active navigation | 3.806 m |
| Single obstacle | VFH / online SLAM | Goal reached | 36.3 s including startup | 4.347 m |
| Corridor | VFH / online SLAM | Goal reached | 24.1 s including startup | 3.802 m |
| U-world | VFH + A* / saved SLAM graph localization | Timeout: `unreachable`; remained stopped | 180 s active trial | approximately zero |

The paired open-field run used the final runtime changes, including occupied-only
memory and scan motion compensation. The corridor run preceded those final
changes; it is a smoke check, not a paired comparison. The U-world map rejection
occurred before local path following, so it does not establish U-route tracking.

ROS interfaces `/map`, TF, `/global_path`, `/local_goal`, `/navigation_state`, and
`/cmd_vel` were exercised. Rosbag recording, fresh simulation launches, SLAM
serialization/localization, structured results, and teardown were exercised through
the actual comparison runner. An interrupted-run shutdown race found during
validation was fixed and covered by a regression test.

## U-world limitation

A conservative 0.05 m raster of the actual U-world SDF supports a collision-checked
13.44 m route with the full 0.43 m safe radius. The saved SLAM map from the
experimental premapping pass does not: replaying its first recorded map finds a
point-robot path but no route even at radius 0.33 m. The runtime correctly reports
`unreachable` and keeps zero velocity. The experimental footprint was not reduced.

This identifies a remaining map/clearance integration issue: the observed SLAM map
does not preserve a robot-width connected corridor in this trial. Review its
coverage, wall reconstruction, and localization before claiming that A* improves
U-world success. Additional premapping viewpoints or SLAM tuning may be needed;
this is not resolved by the offline world-grid result. Every scenario need not
succeed for the runner to record a valid failure.

## Artifacts and reproduction

Commands are in [README.md](README.md). Compact machine-readable results are in
`thesis_test/validation/smoke_results.json` in the workspace. Detailed temporary
artifacts remain under `/tmp/vfh_validation`:

- `run_05/open_field`: paired run, copied configurations, hashes, CSVs, and rosbags.
- `u_run_01/u_world`: the retained U-world failure with map, TF, paths, and states.
- `maps`: the experimentally collected SLAM pose graphs; these are not packaged
  canonical scenario maps and should not be used for thesis statistics without
  inspecting their coverage and alignment.

Earlier temporary runs retain the failures that motivated the fixes. Collision
annotations are null; minimum observed clearance is not collision truth. These
smoke runs establish execution and failure reporting, not statistically reliable
navigation performance or generalization.
