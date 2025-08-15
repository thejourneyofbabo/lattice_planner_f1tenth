# Lattice Planner F1TENTH

Lattice-based path planning for F1TENTH autonomous racing. Generates optimal trajectories in Frenet coordinates with obstacle avoidance.

Part of the F1TENTH autonomous racing system - requires localization and provides paths to the path follower.

## Quick Start

```bash
# Build (from workspace root)
colcon build --packages-select lattice_planner_f1tenth --symlink-install
source install/setup.bash

# For Real Car (default - uses SLAM map mode)
ros2 launch lattice_planner_pkg lattice_planner.launch.py

# For Simulation
ros2 launch lattice_planner_pkg lattice_planner.launch.py sim_mode:=true
```

## Prerequisites

1. **Localization must be running first:**
   - Real car: `ros2 launch particle_filter_cpp localize_slam_launch.py`
   - Simulation: `ros2 launch particle_filter_cpp localize_sim_launch.py`

2. **Robot pose must be initialized** using RViz 2D Pose Estimate button

## Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sim_mode` | `false` | Use simulation topics (`/ego_racecar/odom`) if true |
| `use_sim_time` | `false` | Use simulation time |

## Topic Remapping

### Real Car Mode (`sim_mode:=false`)
- **Input Odometry**: `/pf/pose/odom` (from particle filter)
- **Input Laser**: `/scan`
- **Input Map**: `/map`
- **Output Path**: `/planned_path`

### Simulation Mode (`sim_mode:=true`)
- **Input Odometry**: `/ego_racecar/odom` (from F1TENTH Gym)
- **Input Laser**: `/scan`
- **Input Map**: `/map`
- **Output Path**: `/planned_path`

## Configuration

Edit `config/planner_config.yaml` to adjust:

- **Planning Parameters**: Speed limits, lateral offsets, prediction horizons
- **Reference Paths**: Global path waypoints for different tracks
- **Obstacle Detection**: Collision checking parameters
- **Optimization**: Cost function weights

### Reference Paths

Pre-configured reference paths in `config/reference_paths/`:
- `slam_cent.csv` - Default for SLAM-generated maps
- `Spielberg_global.csv` - Spielberg track (simulation)
- `slam_map_global.csv` - Alternative SLAM reference

## Algorithm Overview

The lattice planner uses a **Frenet coordinate system** for trajectory generation:

1. **Global Path Following**: Tracks pre-defined reference waypoints
2. **Lattice Generation**: Creates multiple trajectory candidates
3. **Collision Checking**: Filters trajectories based on laser scan data
4. **Cost Evaluation**: Selects optimal trajectory based on:
   - Reference path deviation
   - Comfort (acceleration/jerk)
   - Obstacle avoidance
5. **Path Output**: Publishes selected trajectory as `PathWithVelocity`

## Key Topics

**Subscribes:**
- `/odom` or `/ego_racecar/odom` - Robot pose and velocity
- `/scan` - Laser scan for obstacle detection
- `/map` - Occupancy grid map

**Publishes:**
- `/planned_path` - Generated trajectory (`PathWithVelocity` message)
- **Debug Topics** (if enabled):
  - `/lattice_paths` - All generated candidates
  - `/selected_path` - Chosen trajectory
  - `/obstacles` - Detected obstacles

## Integration with System

**Launch Order:**
1. Localization (particle filter)
2. Hardware/Simulation bridge
3. **→ Lattice Planner ←**
4. Path follower

**Data Flow:**
```
Localization → Planner → Path Follower → Vehicle
     ↑             ↑
   Laser        Map Data
```

## Performance

- **Real-time capable**: ~10-20 Hz planning frequency
- **Predictive**: Plans 2-3 seconds ahead
- **Reactive**: Updates based on dynamic obstacles
- **Smooth**: Generates continuous, feasible trajectories

Built for high-speed F1TENTH racing with safety-critical obstacle avoidance.