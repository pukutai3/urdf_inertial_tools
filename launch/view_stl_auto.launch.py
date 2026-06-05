import os
import trimesh
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

MM_TO_M = 0.001


def launch_setup(context, *args, **kwargs):
    # ---- launch arguments ----
    stl_name = LaunchConfiguration('stl').perform(context)
    mass_value = float(LaunchConfiguration('mass').perform(context))
    unit = LaunchConfiguration('unit').perform(context)

    # ---- STL path ----
    stl_dir = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws/src/urdf_xacro_tuner/STL'
    )
    stl_path = os.path.join(stl_dir, stl_name)

    if not os.path.isfile(stl_path):
        raise RuntimeError(f'STL file not found: {stl_path}')

    # ---- mass conversion ----
    if unit == 'g':
        mass_kg = mass_value / 1000.0
    elif unit == 'kg':
        mass_kg = mass_value
    else:
        raise RuntimeError('unit must be g or kg')

    # ---- load STL (mm → m) ----
    mesh = trimesh.load_mesh(stl_path)
    if not mesh.is_watertight:
        raise RuntimeError('STL mesh is not watertight')

    mesh.apply_scale(MM_TO_M)

    density = mass_kg / mesh.volume
    mesh.density = density

    com = mesh.center_mass
    inertia = mesh.moment_inertia

    # ---- print inertial info ----
    print('')
    print('========== Inertial Calculation Result ==========')
    print(f'STL file        : {stl_path}')
    print(f'Mass [kg]       : {mass_kg:.6f}')
    print(f'Center of Mass  : x={com[0]:.6f}, y={com[1]:.6f}, z={com[2]:.6f}')
    print('Inertia Tensor [kg*m^2]')
    print(f'  ixx={inertia[0][0]:.6e}, ixy={inertia[0][1]:.6e}, ixz={inertia[0][2]:.6e}')
    print(f'  iyy={inertia[1][1]:.6e}, iyz={inertia[1][2]:.6e}')
    print(f'  izz={inertia[2][2]:.6e}')
    print('=================================================')    
    print('')  
    print('xacro/ URDF')
    print('    <inertial>')
    print(f'      <origin xyz="{com[0]:.6f} {com[1]:.2f} {com[2]:.6f}"/>')
    print(f'      <mass value="{mass_kg:.6f}"/>')
    print('      <inertia')
    print(f'        ixx="{inertia[0][0]:.6e}" iyy="{inertia[1][1]:.6e}" izz="{inertia[2][2]:.6e}"')
    print(f'        ixy="{inertia[0][1]:.6e}" iyz="{inertia[1][2]:.6e}" ixz="{inertia[0][2]:.6e}"/>')
    print('    </inertial>')
    print('') 
    print('=================================================')
    print('')

    # ---- URDF (semi-transparent white STL) ----
    urdf = f"""
<robot name="stl_auto_view">

  <link name="world"/>

  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="file://{stl_path}" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="white_transparent">
        <color rgba="1.0 1.0 1.0 0.3"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <mesh filename="file://{stl_path}" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}" rpy="0 0 0"/>
      <mass value="{mass_kg:.6f}"/>
      <inertia
        ixx="{inertia[0][0]:.6f}" ixy="{inertia[0][1]:.6f}" ixz="{inertia[0][2]:.6f}"
        iyy="{inertia[1][1]:.6f}" iyz="{inertia[1][2]:.6f}"
        izz="{inertia[2][2]:.6f}"/>
    </inertial>
  </link>

  <joint name="world_to_base" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
  </joint>

</robot>
"""

    pkg_share = get_package_share_directory('urdf_xacro_tuner')
    rviz_config = os.path.join(pkg_share, 'rviz', 'stl.rviz')

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf}],
        ),

        Node(
            package='urdf_xacro_tuner',
            executable='com_marker',
            parameters=[{
                'frame_id': 'base_link',
                'x': float(com[0]),
                'y': float(com[1]),
                'z': float(com[2]),
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stl', default_value='model.stl'),
        DeclareLaunchArgument('mass', default_value='3.2'),
        DeclareLaunchArgument('unit', default_value='g'),
        OpaqueFunction(function=launch_setup),
    ])


