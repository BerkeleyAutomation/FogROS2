# Copyright 2022 The Regents of the University of California (Regents)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
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

import errno
import os
import shutil

_work_dir_cache = None
_instance_dir_cache = None


class MissingEnvironmentVariableException(Exception):
    pass


def _mkdir(path, mode=0o700):
    try:
        os.mkdir(path, mode=mode)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise


def work_dir():
    global _work_dir_cache
    if _work_dir_cache is None:
        home = os.path.expanduser("~")
        path = os.path.join(home, ".fogros2")
        _mkdir(path)
        _work_dir_cache = path
    return _work_dir_cache


def instance_dir():
    global _instance_dir_cache
    if _instance_dir_cache is None:
        path = os.path.join(work_dir(), "instances")
        _mkdir(path)
        _instance_dir_cache = path
    return _instance_dir_cache


def make_zip_file(dir_name, target_path):
    root_dir, workspace_name = os.path.split(dir_name)
    print(root_dir, workspace_name)
    return shutil.make_archive(
        base_dir=workspace_name,
        root_dir=root_dir,
        format="zip",
        base_name=target_path,
    )
