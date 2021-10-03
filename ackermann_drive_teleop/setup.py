from setuptools import setup

package_name = 'ackermann_drive_teleop'

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
    maintainer='olepor',
    maintainer_email='olepo89@gmail.com',
    description='Simple ackermann drive teleoperation node',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ackermann_drive_teleop = ackermann_drive_teleop.keyop:main",
        ],
    },
)
