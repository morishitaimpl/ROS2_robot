import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("raspbot_sim")
    world_path = os.path.join(pkg_share, "worlds", "raspbot_world.sdf")
    model_path = os.path.join(pkg_share, "models", "raspbot", "model.sdf")

    # Start Gazebo GUI + server inside the container (display is handled by start-vnc.sh)
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-v", "4", "-r", world_path],
        output="screen",
    )

    # Spawn robot after Gazebo is up
    spawn = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "source /opt/ros/humble/setup.bash && "
            f"ros2 run ros_gz_sim create -world raspbot_world -file '{model_path}' -name raspbot -x 0 -y 0 -z 0.1",
        ],
        output="screen",
    )

    # Bridge: cmd_vel (ROS->Gazebo), camera image/info (Gazebo->ROS), ultrasonic scan (Gazebo->ROS), clock (Gazebo->ROS)
    bridge = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "source /opt/ros/humble/setup.bash && "
            "ros2 run ros_gz_bridge parameter_bridge "
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock "
            "/model/raspbot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist "
            "/raspbot/camera@sensor_msgs/msg/Image[ignition.msgs.Image "
            "/raspbot/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo "
            "/raspbot/ultrasonic@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        ],
        output="screen",
    )

    # Delay spawning slightly to avoid race on startup
    return LaunchDescription(
        [
            gazebo,
            TimerAction(period=3.0, actions=[spawn]),
            TimerAction(period=3.5, actions=[bridge]),
        ]
    )

