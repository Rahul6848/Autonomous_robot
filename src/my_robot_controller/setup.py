from setuptools import find_packages, setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rahul',
    maintainer_email='rahul@todo.todo',
    description='Python controller nodes for my_robot.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Original nodes
            'move_robot = my_robot_controller.move_robot:main',
            'perception_controller = my_robot_controller.perception_controller:main',
            'lidar_navigation_controller = my_robot_controller.lidar_navigation_controller:main',
            # New command-based mission nodes
            'mission_controller = my_robot_controller.mission_controller:main',
            'mission_commander  = my_robot_controller.mission_commander:main',
            'parking_mission_controller = my_robot_controller.parking_mission_controller:main',
        ],
    },
)
