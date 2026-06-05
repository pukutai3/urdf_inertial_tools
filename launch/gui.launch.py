from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    urdf = LaunchConfiguration('urdf').perform(context).strip()
    package_root = LaunchConfiguration('package_root').perform(context).strip()

    arguments = []
    if urdf:
        arguments.extend(['--urdf', urdf])
    if package_root:
        arguments.extend(['--package-root', package_root])

    return [
        Node(
            package='urdf_xacro_tuner',
            executable='urdf-xacro-tuner-gui',
            name='urdf_xacro_tuner_gui',
            output='screen',
            emulate_tty=True,
            arguments=arguments,
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('urdf', default_value='', description='URDF or xacro path to open at startup'),
            DeclareLaunchArgument('package_root', default_value='', description='Optional package root override'),
            OpaqueFunction(function=launch_setup),
        ]
    )
