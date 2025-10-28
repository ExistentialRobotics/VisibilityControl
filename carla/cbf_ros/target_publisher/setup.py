# setup.py
from setuptools import setup

package_name = 'target_publisher'

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
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS 2 package to publish target pose and velocity from a trajectory file.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'target_publisher_node = target_publisher.target_publisher_node:main'
        ],
    },
)

