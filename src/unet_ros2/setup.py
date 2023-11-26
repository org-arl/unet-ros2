from setuptools import find_packages, setup

package_name = 'unet_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mandar Chitre',
    maintainer_email='mandar@no.reply',
    description='UnetStack ROS2 gateway',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unet_gw = unet_ros2.unet_gw:main'
        ],
    },
)
