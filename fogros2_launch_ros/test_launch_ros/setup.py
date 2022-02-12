from setuptools import find_packages
from setuptools import setup

package_name = 'test_launch_ros'

setup(
    name=package_name,
    version='0.17.0',
    packages=find_packages(exclude=['test']),
    install_requires=[
        'setuptools',
        'demo_nodes_py',
        'launch_ros',
        'pyyaml',
    ],
    zip_safe=True,
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    author='William Woodall',
    author_email='william@osrfoundation.org',
    maintainer='Aditya Pande, Jacob Perron, Michel Hidalgo',
    maintainer_email='aditya.pande@openrobotics.org, jacob@openrobotics.org, michel@ekumenlabs.com',  # noqa: E501
    url='https://github.com/ros2/launch',
    download_url='https://github.com/ros2/launch/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Tests for ROS specific extensions to `launch`.',
    long_description=(
        'This package provides tests for the ROS specific '
        'extensions to the launch package.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
)
