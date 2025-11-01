from setuptools import setup
import os

package_name = 'ugv_dashboard_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'PySide6', 'rclpy', 'std_msgs'],
    zip_safe=True,
    maintainer='liv',
    maintainer_email='liv.alvz@gmail.com',
    description='PySide6 GUI to monitor UGV telemetry data',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gui_dashboard = ugv_dashboard_gui.pyside6_gui.telemetry_dashboard_gui:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gui_launch.py']),
    ],
)
