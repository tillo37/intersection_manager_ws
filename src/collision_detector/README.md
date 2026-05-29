# collision_detector (Member 7 Mikhail)

Monitors every vehicle-pedestrian pair independently at 10 Hz.
Slows a vehicle when within 2.0 m of a pedestrian, stops it completely when within 1.0 m.
Publishes colour-coded warning spheres to RViz.

## ROS Functions

- `check_proximity()` — Timer callback at 10 Hz. Calculates the distance from each
  vehicle to its closest pedestrian and classifies it as safe, warning, or danger.
  Calls `broadcast_warning()` with the resulting index lists and publishes RViz markers.
- `broadcast_warning()` — Publishes vehicle indices to `/collision_warning` (Int32MultiArray)
  for vehicles that must stop (< 1.0 m) and to `/collision_slow` (Int32MultiArray)
  for vehicles that must slow to 50% speed (1.0–2.0 m).

## Run

```bash
ros2 run collision_detector collision_detector_node