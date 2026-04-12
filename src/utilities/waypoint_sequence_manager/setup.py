from setuptools import setup

package_name = 'waypoint_sequence_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/waypoint_sequence_manager.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ji Zhang',
    maintainer_email='zhangji@cmu.edu',
    description='Waypoint Sequence Manager for Factory Navigation',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_sequence_manager = waypoint_sequence_manager.waypoint_sequence_manager:main'
        ],
    },
)