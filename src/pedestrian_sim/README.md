# pedestrian_sim (Member 3)

Simulates 3 pedestrians with random walks. Fires emergency stop when any enters the 1.5 m danger zone.

## ROS Functions
- `simulate_pedestrians()` — Publishes `/obstacles/pose` (PoseArray) at 2 Hz.
- `trigger_emergency_stop()` — Publishes `True` to `/emergency_stop` (Bool) when danger detected.

## Run
```bash
ros2 run pedestrian_sim pedestrian_sim_node
```
