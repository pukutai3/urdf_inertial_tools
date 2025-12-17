from setuptools import setup

package_name = 'urdf_inertial_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/view_stl_auto.launch.py']),
        ('share/' + package_name + '/rviz',
         ['rviz/stl.rviz']),
    ],
    install_requires=[
        'setuptools',
        'trimesh',
        'numpy',
    ],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='STL inertial visualization tools',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'com_marker = urdf_inertial_tools.com_marker_node:main',
        ],
    },
)

