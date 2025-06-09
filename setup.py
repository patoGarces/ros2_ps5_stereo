from setuptools import find_packages, setup

package_name = 'ros2_ps5_stereo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/ros2_ps5_stereo/launch', ['launch/ros2_ps5_stereo_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pato Garcés',
    maintainer_email='patricio.garces@outlook.com',
    description='Stereo camera for ps5 camera',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = ros2_ps5_stereo.camera_node:main'
        ],
    },
)
