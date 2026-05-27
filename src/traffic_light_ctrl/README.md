# traffic_light_ctrl (Member 2)

Cycles RED → GREEN → YELLOW automatically. Exposes `/set_phase` for emergency overrides.

## ROS Functions
- `cycle_phase()` — Advances phase on a timer, publishes to `/traffic/phase` (String).
- `set_phase_callback()` — Service server on `/set_phase` (SetBool), forces GREEN on request.

## Run
```bash
ros2 run traffic_light_ctrl traffic_light_node
```
