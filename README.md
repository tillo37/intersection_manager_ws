# Autonomous Urban Intersection Manager

ROS2 (Humble) simulation of a smart intersection manager for Ubuntu 22.04.

## Prerequisites
```bash
sudo apt install ros-humble-desktop python3-colcon-common-extensions python3-pip
pip3 install click pytest
```

## Build
```bash
cd intersection_manager_ws
colcon build
source install/setup.bash
```

## Run everything
```bash
# All nodes + RViz2
ros2 launch launch/full_system.launch.py

# All nodes, no RViz
ros2 launch launch/full_system.launch.py rviz:=false
```

## CLI (Member 5)
```bash
# Show running nodes and topics
python3 src/cli_dashboard/cli_dashboard/cli_dashboard.py status

# Force traffic phase to GREEN
python3 src/cli_dashboard/cli_dashboard/cli_dashboard.py set-phase --phase GREEN

# Record 30 seconds of all topics
python3 src/cli_dashboard/cli_dashboard/cli_dashboard.py record --duration 30

# Publish emergency stop
python3 src/cli_dashboard/cli_dashboard/cli_dashboard.py estop
```

## Tests
```bash
pytest tests/test_intersection.py -v
```

## Play back a bag
```bash
ros2 bag play bags/scenario
```

## RViz
Open RViz2 and add a **MarkerArray** display on topic `/viz/markers`.  
The intersection zone is colour-coded: 🟢 GREEN / 🟡 YELLOW / 🔴 RED / ⚠️ Emergency.

## Architecture
```
vehicle_control  ──/vehicle/pose──────────────────────────▶ intersection_manager
                 ──/vehicle/velocity──────────────────────▶ intersection_manager
                 ◀──/grant_access (service)────────────────  intersection_manager

traffic_light    ──/traffic/phase──────────────────────────▶ intersection_manager
                 ◀──/set_phase (service)────────────────────  intersection_manager

pedestrian_sim   ──/obstacles/pose─────────────────────────▶ intersection_manager
                 ──/emergency_stop──────────────────────────▶ intersection_manager

intersection_manager ──/viz/markers────────────────────────▶ RViz2
```

## Packages
| Package | Member | Node |
|---------|--------|------|
| `vehicle_control` | M1 | `vehicle_control_node` |
| `traffic_light_ctrl` | M2 | `traffic_light_node` |
| `pedestrian_sim` | M3 | `pedestrian_sim_node` |
| `intersection_manager` | M4 | `intersection_manager_node` |
| `cli_dashboard` | M5 | CLI + tests + launch |
