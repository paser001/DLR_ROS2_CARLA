from setuptools import find_packages, setup

package_name = 'carla_parking_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='pa.serra001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		    'carla_interface_node = carla_parking_nodes.carla_interface_node:main',
       	 	'parking_map_node = carla_parking_nodes.parking_map_node:main',
            'sensor_viewer_node = carla_parking_nodes.sensor_viewer_node:main',
            'keyboard_control_node = carla_parking_nodes.keyboard_control_node:main',
            'parking_controller_node = carla_parking_nodes.parking_controller_node:main',
            'joystick_control_node = carla_parking_nodes.joystick_control_node:main',
        ],
    },
)
