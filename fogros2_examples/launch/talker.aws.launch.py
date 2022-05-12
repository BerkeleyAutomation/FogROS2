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

from launch_ros.actions import Node

from fogros2 import AWS, CloudNode, FogROSLaunchDescription


def ami_image():
    # An AMI is an Amazon Web Services virtual machine image with a
    # pre-installed OS and dependencies.  We match the AMI in the
    # cloud to have the same OS release as the robot.  Currently we
    # support Ubuntu 20.04 and 22.04.

    import lsb_release

    ubuntu_release = lsb_release.get_os_release()["RELEASE"]

    if ubuntu_release == "20.04":
        return "ami-00f25057ddc9b310b"
    if ubuntu_release == "22.04":
        return "ami-0b6030c78f8b2f076"

    raise ValueError(f"No AMI for {ubuntu_release}")


def generate_launch_description():
    """
    Talker example that launches the listener on AWS.
    """
    ld = FogROSLaunchDescription()
    machine1 = AWS(
        region="us-west-1", ec2_instance_type="t2.micro", ami_image=ami_image()
    )

    listener_node = Node(
        package="fogros2_examples", executable="listener", output="screen"
    )

    talker_node = CloudNode(
        package="fogros2_examples",
        executable="talker",
        output="screen",
        machine=machine1,
    )
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    return ld
