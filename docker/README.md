# Docker Setup for Autonomy Stack Vega

Docker configurations for running autonomy_stack_vega in containers.

## Images

| Architecture | Base Image | Use Case |
|-------------|------------|----------|
| AMD64 | `osrf/ros:jazzy-desktop-full` | Intel/AMD processors |
| ARM64 | `arm64v8/ros:jazzy-ros-base` | Jetson, ARM processors |

Both include ROS2 Jazzy, CycloneDDS, SuperOdom dependencies, and Livox SDK2.

## Quick Start

```bash
# One-time host setup
cd docker && ./setup_host.sh

# Start container (replace amd64 with arm64 for ARM systems)
docker compose --profile amd64 up -d
docker compose exec autonomy-amd64 bash

# Inside container
cd /workspace
colcon build --symlink-install
source install/setup.bash
ros2 launch vehicle_simulator system_real_robot.launch.py
```

## Container Management

```bash
docker compose --profile <arch> up -d      # Start
docker compose --profile <arch> down       # Stop
docker compose --profile <arch> down -v    # Stop and remove volumes
docker compose exec autonomy-<arch> bash   # Enter container
docker compose build autonomy-<arch>       # Build image
```

## Filesystem

| Host | Container | Purpose |
|------|-----------|---------|
| `../` (project root) | `/workspace` | Source code |
| Named volumes | `/workspace/{build,install,log}` | Build artifacts |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | GUI support |

## Network

- **Mode**: `network_mode: host` for direct LiDAR access and ROS2 DDS discovery
- **CycloneDDS**: Pre-configured with large buffer settings (`rmem_max=2147483647`)
- **GUI**: Run `xhost +local:docker` on host before using RViz

## Troubleshooting

```bash
# GUI not working
xhost +local:docker

# Build issues - clean and rebuild
rm -rf /workspace/build /workspace/install /workspace/log
colcon build --symlink-install

# Check CycloneDDS
sysctl net.core.rmem_max  # Should be 2147483647
```
