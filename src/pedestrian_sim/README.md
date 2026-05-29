# pedestrian_sim (Member 3  Temurjon)

Simulates 6 pedestrians with random walks biased toward the intersection centre.
Fires emergency stop when any pedestrian enters the 1.0 m danger zone.

## ROS Functions

- `simulate_pedestrians()` — Timer callback at 2 Hz. Moves all pedestrians,
  publishes `/obstacles/pose` (PoseArray), and calls `trigger_emergency_stop()`
  when any pedestrian enters the danger zone.
- `trigger_emergency_stop()` — Publishes `True` to `/emergency_stop` (Bool)
  and logs a warning when a pedestrian is within 1.0 m of the intersection centre.

## Run

```bash
ros2 run pedestrian_sim pedestrian_sim_node
```
