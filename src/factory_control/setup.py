from glob import glob

from setuptools import find_packages, setup

package_name = 'factory_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fares',
    maintainer_email='fares@example.com',
    description='IK and control nodes for the Smart Mini Factory simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawner_node = factory_control.spawner_node:main',
            'conveyor_io_node = factory_control.conveyor_io_node:main',
            'arm_node = factory_control.arm_node:main',
            'grasp_trial = factory_control.grasp_trial:main',
            'calibrate_node = factory_control.calibrate_node:main',
            'vision_node = factory_control.vision_node:main',
            'supervisor_node = factory_control.supervisor_node:main',
            'logger_node = factory_control.logger_node:main',
            'report = factory_control.logger_node:report_main',
            'pose_relay_node = factory_control.pose_relay_node:main',
            'vision_trial = factory_control.vision_trial:main',
        ],
    },
)
