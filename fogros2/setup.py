# Copyright ©2022. The Regents of the University of California (Regents).
# All Rights Reserved. Permission to use, copy, modify, and distribute this
# software and its documentation for educational, research, and not-for-profit
# purposes, without fee and without a signed licensing agreement, is hereby
# granted, provided that the above copyright notice, this paragraph and the
# following two paragraphs appear in all copies, modifications, and
# distributions. Contact The Office of Technology Licensing, UC Berkeley, 2150
# Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620, (510) 643-7201,
# otl@berkeley.edu, http://ipira.berkeley.edu/industry-info for commercial
# licensing opportunities. IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY
# FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES,
# INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE. REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY,
# PROVIDED HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
# MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "fogros2"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "configs"),
            glob("configs/*.xml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Kaiyuan (Eric) Chen, Víctor Mayoral-Vilches",
    author_email="kych@berkeley.edu, v.mayoralv@gmail.com",
    maintainer="Kaiyuan (Eric) Chen",
    maintainer_email="kych@berkeley.edu",
    description="ROS 2 extension for cloud computational graph deployment",
    long_description="""A ROS 2 extension for the cloud deployment
                        of computational graphs in a cloud-provider agnostic
                        and security-conscious manner.""",
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
        ],
    },
)
