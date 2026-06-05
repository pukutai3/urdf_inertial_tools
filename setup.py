from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

package_name = 'urdf_xacro_tuner'
base_dir = Path(__file__).resolve().parent

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml']),
        ('share/' + package_name + '/launch', glob(str(base_dir / 'launch' / '*.py'))),
        ('share/' + package_name + '/rviz', glob(str(base_dir / 'rviz' / '*.rviz'))),
        ('share/' + package_name + '/STL', glob(str(base_dir / 'STL' / '*.stl'))),
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
