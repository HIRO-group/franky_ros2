from setuptools import find_packages, setup

package_name = 'franky_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'franky'],
    zip_safe=True,
    maintainer='caleb',
    maintainer_email='carson.kohlbrenner@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'franky_joint_velocity_controller = franky_ros2.franky_joint_velocity_controller:main',
            'franky_joint_position_controller = franky_ros2.franky_joint_position_controller:main'
        ],
    },
)
