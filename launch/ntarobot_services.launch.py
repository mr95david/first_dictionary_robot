# Importe de librerias de ros, configuracion de launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
    # Declaracion de configuracion
    val_initial_pose = LaunchConfiguration("val_pose_initial")

    # Declaracion de argumento
    val_inital_cmd = DeclareLaunchArgument(
        "val_pose_initial",
        default_value = "true",
        description = "Use initial pose from default lab"
    )

    # Descripcion de nodos de servicio
    service_node_state = Node(
        package = "ntarobot_services",
        executable = "robot_state.py",
        name = "move_to_pose",
        output = "screen"
    )

    service_move_abs = Node(
        package = "ntarobot_services",
        executable = "move_pose.py",
        name = "move_to_pose",
        output = "screen"
    )
    service_move_relative = Node(
        package = "ntarobot_services",
        executable = "relative_pose.py",
        name = "relative_pose",
        output = "screen"
    )

    # Ejecucion de servicio general
    service_initial_pose = ExecuteProcess(
        cmd = [[
            FindExecutable(name = 'ros2'),
            " service call ",
            "/set_initial_pose ",
            "nav2_msgs/srv/SetInitialPose ",
            '"{pose: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: \'map\'}, \
pose: {pose: {position:{x: -0.215689, y: -0.154802, z: 0.0}, \
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, \
covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}}"'
        ]],
        shell = True,
        condition = IfCondition(val_initial_pose)
    )

    service_cancell_all = Node(
        package = "ntarobot_services",
        executable = "stop_all.py",
        name = "stop_all_movements",
        output = "screen"
    )

    # Ejecucion final de servicios
    return LaunchDescription([
        val_inital_cmd,
        service_node_state,
        service_move_abs,
        service_move_relative,
        service_initial_pose,
        service_cancell_all
    ])