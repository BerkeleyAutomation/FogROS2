from glob import glob

from setuptools import find_packages, setup

package_name = "fogros2"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/configs", glob("configs/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Kaiyuan (Eric) Chen, Víctor Mayoral-Vilches",
    author_email="kych@berkeley.edu, v.mayoralv@gmail.com",
    maintainer="Kaiyuan (Eric) Chen",
    maintainer_email="kych@berkeley.edu",
    description="A ROS 2 extension for the cloud deployment of computational graphs in a cloud-provider agnostic and security-conscius manner.",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "ros2cli.command": [
            "fog = fogros2.command.fog:FogCommand",
        ],
        "fogros2.verb": [
            "list = fogros2.verb.list:ListVerb",
            "delete = fogros2.verb.delete:DeleteVerb",
            "connect = fogros2.verb.ssh:SSHVerb",
            "image = fogros2.verb.image:ImageVerb",
        ],
    },
)
