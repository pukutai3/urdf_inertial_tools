import os
from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

package_name = 'urdf_xacro_tuner'
base_dir = Path(__file__).resolve().parent


def source_dir() -> Path:
    candidates = [base_dir, *base_dir.parents, Path.cwd(), *Path.cwd().parents]
    for candidate in candidates:
        direct = candidate / package_name
        if (candidate / 'launch').exists() and (candidate / 'package.xml').exists():
            return candidate
        if (direct / 'launch').exists() and (direct / 'package.xml').exists():
            return direct
        nested = candidate / 'src' / 'urdf_inertial_tools'
        if (nested / 'launch').exists() and (nested / 'package.xml').exists():
            return nested
    return base_dir


def relpaths(pattern: str) -> list[str]:
    root = source_dir()
    files = sorted(root.glob(pattern))
    return [os.path.relpath(path, Path.cwd()) for path in files]

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml']),
        ('share/' + package_name + '/launch', relpaths('launch/*.py')),
        ('share/' + package_name + '/rviz', relpaths('rviz/*.rviz')),
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
