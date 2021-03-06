from setuptools import setup

package_name = 'rover_waypoint'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='devansh@umich.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_node = rover_waypoint.hello:main',
            'move_forward = rover_waypoint.move_rover_forward:main',
            'joystick_safety = rover_waypoint.safety:main',
            'move_waypoint = rover_waypoint.move_waypoint:main'
        ],
    },
)
