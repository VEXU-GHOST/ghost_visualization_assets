"""Launch Baby Jerry inside a static field visualization.

The chassis visual is on the physical frame link so the importer retains it.
The field is static visual scenery. The world has no gravity or field collisions.

Targets webots_ros2_driver 2025.0.0. URDFSpawner is deprecated in that release
but remains available. This starter does not implement joint commands or reset
recovery. Close the launch and relaunch after resetting Webots.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from webots_ros2_driver.urdf_spawner import URDFSpawner
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    assets = Path(get_package_share_directory("ghost_visualization_assets"))
    urdf = assets / "baby_jerry" / "urdf" / "baby_jerry_webots.urdf"
    world = assets / "WeBots" / "worlds" / "baby_jerry_field.wbt"

    webots = WebotsLauncher(world=str(world), ros2_supervisor=True)
    spawn = URDFSpawner(
        name="BabyJerry",
        urdf_path=str(urdf),
        translation="0.65 0.65 0.3",
        output="screen",
    )
    # The driver waits for a robot with this name to become available.
    driver = WebotsController(
        robot_name="BabyJerry",
        parameters=[{"robot_description": str(urdf), "use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        spawn,
        driver,
        RegisterEventHandler(OnProcessExit(
            target_action=webots,
            on_exit=[EmitEvent(event=Shutdown(reason="Webots closed"))],
        )),
    ])
