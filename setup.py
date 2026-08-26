from setuptools import setup

package_name = 'robo_roomba'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/open_loop_launch.py',
            'launch/standard_launch.py',
            'launch/advanced_launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linus',
    maintainer_email='s6limall@uni-bonn.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'open_loop_controller = robo_roomba.open_loop_controller:main',
            'wall_detection = robo_roomba.wall_detection:main',
            'wall_avoidance = robo_roomba.wall_avoidance:main',
            'robo_roomba_controller = robo_roomba.robo_roomba_controller:main',
            'robo_roomba_controller_v2 = robo_roomba.robo_roomba_controller_v2:main',
        ],
    },
)
