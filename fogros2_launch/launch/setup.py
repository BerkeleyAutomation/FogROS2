from setuptools import find_packages
from setuptools import setup

package_name = 'launch'

setup(
    name=package_name,
    version='0.21.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/launch/frontend', ['share/launch/frontend/grammar.lark']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Dirk Thomas',
    author_email='dthomas@osrfoundation.org',
    maintainer='Aditya Pande, Michel Hidalgo',
    maintainer_email='aditya.pande@openrobotics.org, michel@ekumenlabs.com',
    url='https://github.com/ros2/launch',
    download_url='https://github.com/ros2/launch/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Launch processes specified in launch files.',
    long_description=(
        'This package provides the ability to run multiple '
        'processes and react on individual processes exiting.'),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
)
