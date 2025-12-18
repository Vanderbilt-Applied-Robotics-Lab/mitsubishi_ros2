from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Arguments
    rviz = LaunchConfiguration("rviz").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    robot_controller = LaunchConfiguration("robot_controller").perform(context)
    simulation = LaunchConfiguration("simulation").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    robot_port = LaunchConfiguration("robot_port").perform(context)
    control_port = LaunchConfiguration("control_port").perform(context)
    robot_password = LaunchConfiguration("robot_password").perform(context)

    # URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("mitsubishi_robot_description"),
                    "urdf",
                    robot_name,
                    f"{robot_name}.xacro",
                ]
            ),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "robot_port:=",
            robot_port,
            " ",
            "control_port:=",
            control_port,
            " ",
            "robot_password:=",
            robot_password,
            " ",
            "simulation:=",
            simulation,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Controllers
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("mitsubishi_robot_bringup"),
         "config", "controllers.yaml"]
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("mitsubishi_robot_bringup"),
            "config",
            "rviz",
            "view.rviz",
        ]
    )

    # Nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "-c", "/controller_manager"],
    )

    # Launch order
    nodes = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner_node,
        robot_controller_spawner_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner_node,
                on_exit=[rviz_node],
            ),
        ),
    ]

    return nodes


def generate_launch_description():
    # Arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="rv_6sdl_s15",
            description="Selected robot model number, separated by underscores (Ex: 'rv_6sdl_s15')",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="position_controller",
            description="Selected robot controller, options can be found in 'mitsubishi_robot_bringup/config/controllers.yaml'",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "simulation",
            default_value="10003",
            description="Port of the hos computer",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.0.1",
            description="IP address of the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_port",
            default_value="10000",
            description="Port of the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "control_port",
            default_value="10003",
            description="Port of the hos computer",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_password",
            default_value="ARMALABS",
            description="Password to use in startup negotiation with robot",
        )
    )

    # Launch function
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
