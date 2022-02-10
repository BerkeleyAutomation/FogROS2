from setuptools import find_packages
from setuptools import setup

package_name = 'ros2launch'

setup(
    name=package_name,
    version='0.17.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    author='William Woodall',
    author_email='william@osrfoundation.org',
    maintainer='Aditya Pande, Jacob Perron, Michel Hidalgo',
    maintainer_email='aditya.pande@openrobotics.org, jacob@openrobotics.org, michel@ekumenlabs.com',  # noqa: E501
    url='https://github.com/ros2/launch/tree/master/ros2launch',
    download_url='https://github.com/ros2/launch/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='The launch command for ROS 2 command line tools.',
    long_description="""\
The package provides the launch command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'launch = ros2launch.command.launch:LaunchCommand',
        ],
    }
)
