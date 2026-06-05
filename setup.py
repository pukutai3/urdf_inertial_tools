from glob import glob
from setuptools import find_packages, setup

package_name = 'urdf_xacro_tuner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=[
        'setuptools',
        'trimesh',
        'numpy',
    ],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='URDF/xacro mass, inertia, and joint tuning tools',
    license='MIT',
    entry_points={
        'console_scripts': [
            'urdf-xacro-tuner = urdf_xacro_tuner.gui:main',
            'urdf-xacro-tuner-gui = urdf_xacro_tuner.gui:main',
            'urdf-xacro-tuner-cli = urdf_xacro_tuner.urdf_mass_inertia:main',
            'urdf-inertia = urdf_xacro_tuner.urdf_mass_inertia:main',
            'urdf-inertia-gui = urdf_xacro_tuner.gui:main',
        ],
    },
)
