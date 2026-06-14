import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ekf_experiment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
      
            'trajectory_planner = ekf_experiment.trajectory_planner_node:main',
            'kinematic_sim = ekf_experiment.kinematic_sim_node:main',
            'sensor_sim = ekf_experiment.sensor_sim_node:main',
            'ekf_fusion = ekf_experiment.ekf_fusion_node:main',
            'gazebo_driver = ekf_experiment.gazebo_driver_node:main',
       ],
    },
)
