# ghost_visualization_assets

Laptop preview launchers and robot and field geometry for GHOST visualization.
This repository is also a
ROS 2 `ament_cmake` package, cloned and built separately on the visualization
laptop. It is not a VEXU_GHOST submodule and is not needed on the robot.

## Baby Jerry

`baby_jerry/urdf/baby_jerry.urdf` defines the links, joints, and mesh placement.
`baby_jerry/meshes/` contains 25 binary STL meshes, with coordinates in meters.
The model uses the reduced meshes and excludes the tank canister visual.
Its three continuous joints are `revolute_1`, `revolute_2`, and `revolute_3`.

Meshes use Git LFS. Install Git LFS before checking out this repository, then run:

```bash
git lfs install --local
git lfs pull
```

## Laptop setup (ROS 2 Humble)

This preview is self-contained; it does not require a VEXU_GHOST checkout.
For a checkout at `~/ghost_visualization_assets`, install the preview
dependencies on the laptop:

```bash
sudo apt-get install git-lfs ros-humble-ament-index-python ros-humble-launch \
  ros-humble-launch-ros ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-foxglove-bridge
git -C ~/ghost_visualization_assets lfs install --local
git -C ~/ghost_visualization_assets lfs pull
```

Build the visualization package in a separate laptop workspace:

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ghost_visualization_ws
cd ~/ghost_visualization_ws
colcon build --base-paths "$HOME/ghost_visualization_assets" \
  --packages-select ghost_visualization_assets
source install/setup.bash
ros2 launch ghost_visualization_assets baby_jerry_preview.launch.py gui:=true
```

In another laptop terminal, start Foxglove Bridge:

```bash
source /opt/ros/humble/setup.bash
source ~/ghost_visualization_ws/install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Connect Foxglove to `ws://localhost:8765` and select `/robot_description` as
the URDF source. This is a local preview; robot telemetry integration is a
separate step.

The preview launcher starts robot_state_publisher and wheel sliders. Use
`publish_joint_states:=false` when another node supplies real joint feedback.
Start Foxglove Bridge from the same sourced workspace so it can resolve mesh
`package://` URLs. For the local preview, use `root` as the 3D display frame
and Z as the mesh up axis. The preview does not publish an odometry connection.

## Simplification tool

`tools/simplify_meshes.py` makes a separate preview copy, reduces the three
largest meshes, and writes size and geometry comparison reports. It preserves
mesh coordinates and leaves the input files untouched. Supply the original
full-resolution model directory, containing `meshes/` and exactly one URDF
under `urdf/`; the checked-in model is already reduced.

Install its optional dependencies in a Python virtual environment:

```bash
python3 -m venv .venv
.venv/bin/pip install -r tools/requirements.txt
.venv/bin/python tools/simplify_meshes.py --help
```

Future `main_robot/` and `field/` directories will also be installed by this
package when added.
