import os
from glob import glob

from setuptools import setup

package_name = "fogros2_examples"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Kaiyuan (Eric) Chen, Víctor Mayoral-Vilches",
    author_email="kych@berkeley.edu, v.mayoralv@gmail.com",
    maintainer="Kaiyuan (Eric) Chen",
    maintainer_email="kych@berkeley.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "talker = fogros2_examples.talker:main",
            "listener = fogros2_examples.listener:main",
        ],
    },
)
