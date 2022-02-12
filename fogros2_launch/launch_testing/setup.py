import glob

from setuptools import find_packages
from setuptools import setup


setup(
    name='launch_testing',
    version='0.21.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/launch_testing']),
        ('lib/launch_testing', glob.glob('example_processes/**')),
        ('share/launch_testing', ['package.xml']),
        ('share/launch_testing/examples', glob.glob('test/launch_testing/examples/[!_]**')),
    ],
    entry_points={
        'console_scripts': ['launch_test=launch_testing.launch_test:main'],
        'pytest11': ['launch_testing = launch_testing.pytest.hooks'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    author='Pete Baughman, Dirk Thomas, Esteve Fernandez',
    author_email='pete.baughman@apex.ai, dthomas@osrfoundation.org',
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
    description='Create tests which involve launch files and multiple processes.',
    long_description=('A package to create tests which involve'
                      ' launch files and multiple processes.'),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
)
