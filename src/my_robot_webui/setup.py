from glob import glob
from setuptools import setup

package_name = 'my_robot_webui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_data={package_name: ['static/*']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/static', glob('my_robot_webui/static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rahul',
    maintainer_email='rahul@todo.todo',
    description='Browser dashboard for launching and controlling my_robot.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_server = my_robot_webui.dashboard_server:main',
        ],
    },
)
