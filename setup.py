from setuptools import find_packages, setup

package_name = 'craaybot_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eric C.',
    maintainer_email='eriico39hi@gmail.com',
    description='2 Wheeled Robot control system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serialbr = craaybot_ros2.serialbr_pid:main',
            'kinematics = craaybot_ros2.kinematics:main'
        ],
    },
)
