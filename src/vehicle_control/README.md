# vehicle_control (Member 1)

Simulates a vehicle driving toward the intersection.

## ROS Functions
- `publish_state()` — Publishes `/vehicle/pose` (PoseStamped) and `/vehicle/velocity` (Twist) at 10 Hz.
- `request_access()` — Calls `/grant_access` (SetBool) service when within 2 m of intersection.

## Run
```bash
ros2 run vehicle_control vehicle_control_node
```
