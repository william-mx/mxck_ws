from setuptools import setup
import os
from glob import glob

package_name = 'vehicle_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
            'console_scripts': [
                    'rc_to_joy = vehicle_control.rc_to_joy:main',
                    'joy_to_ackermann = vehicle_control.joy_to_ackermann:main',
                    'ackermann_to_vesc = vehicle_control.ackermann_to_vesc:main',
            ],
    },
)

