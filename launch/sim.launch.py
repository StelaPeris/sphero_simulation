import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("sphero_simulation"), "config", "sim.yaml")
    bcfg = os.path.join(get_package_share_directory("sphero_simulation"), "config", "sim_bridge.yaml")
    rid = LaunchConfiguration("robot_id")
    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="10"),
        DeclareLaunchArgument("model", default_value="pendulum"),
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("variant", default_value="drive"),
        DeclareLaunchArgument("world", default_value="arena"),
        DeclareLaunchArgument("bridge", default_value="true"),     # sim_bridge (robot_bridge za simulaciju)
        DeclareLaunchArgument("ball_id", default_value="0"),
        DeclareLaunchArgument("yaw_loop", default_value="false"),  # true -> /robot<ID>/yaw_target vozi smjer
        DeclareLaunchArgument("yaw_offset", default_value="0.0"),  # IMU yaw = svijet + offset (kao yaw_offset u robot_config.json)
        DeclareLaunchArgument("x0", default_value="0.0"),          # pocetna poza kugle u svijetu
        DeclareLaunchArgument("y0", default_value="0.0"),
        DeclareLaunchArgument("yaw0", default_value="0.0"),        # pocetni smjer voznje u svijetu [deg]
        Node(
            package="sphero_simulation",
            executable="sim_node",
            name="sphero_sim",
            namespace=PythonExpression(["'robot' + '", rid, "'"]),
            output="screen",
            parameters=[cfg, {
                "model": LaunchConfiguration("model"),
                "variant": LaunchConfiguration("variant"),
                "world": LaunchConfiguration("world"),
                "camera_topic": PythonExpression(["'/b' + '", LaunchConfiguration("ball_id"), "' + '/pose'"]),
                "yaw_offset_deg": PythonExpression(["float(", LaunchConfiguration("yaw_offset"), ")"]),
                "init_x": PythonExpression(["float(", LaunchConfiguration("x0"), ")"]),
                "init_y": PythonExpression(["float(", LaunchConfiguration("y0"), ")"]),
                "init_yaw_deg": PythonExpression(["float(", LaunchConfiguration("yaw0"), ")"]),
                "viewer": PythonExpression(["'", LaunchConfiguration("viewer"), "'.lower() == 'true'"]),
            }],
        ),
        Node(
            package="sphero_simulation",
            executable="sim_bridge",
            name="sim_bridge",
            output="screen",
            condition=IfCondition(LaunchConfiguration("bridge")),
            parameters=[bcfg, {
                "robot_id": PythonExpression([rid]),
                "ball_id": PythonExpression([LaunchConfiguration("ball_id")]),
                "yaw_loop": PythonExpression(["'", LaunchConfiguration("yaw_loop"), "'.lower() == 'true'"]),
            }],
        ),
    ])