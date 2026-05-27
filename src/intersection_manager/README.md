# intersection_manager (Member 4)

Brain of the system. Subscribes to all topics, arbitrates access, overrides traffic light in emergencies, publishes RViz markers.

## ROS Functions
- `vehicle_pose_cb()` — Subscriber on `/vehicle/pose`; logs vehicles within 2 m.
- `arbitrate_access()` — Service server on `/grant_access` (SetBool); grants/denies based on phase + emergency state.

## Run
```bash
ros2 run intersection_manager intersection_manager_node
```

## RViz
Add a MarkerArray display, topic `/viz/markers`.
