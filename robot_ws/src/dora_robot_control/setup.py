from setuptools import find_packages, setup

package_name = 'dora_robot_control'

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
    maintainer='moustapha',
    maintainer_email='m.azaimi@student.avans.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_node = dora_robot_control.serial_node:main',
            'odometry_node = dora_robot_control.odometry_node:main',
        ],
    },
)
