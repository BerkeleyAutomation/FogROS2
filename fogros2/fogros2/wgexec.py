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


import logging
import shlex
import subprocess


logger = logging.getLogger(__name__)


def execute(command, input=None, suppressoutput=False, suppresserrors=False):
    """Execute a command"""
    args = shlex.split(command)
    stdin = None if input is None else subprocess.PIPE
    input = None if input is None else input.encode('utf-8')
    nsp = subprocess.Popen(args, stdin=stdin, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = nsp.communicate(input=input)
    if err is not None:
        err = err.decode('utf8')
        if not suppresserrors and (len(err) > 0):
            logger.error(err)
    out = out.decode('utf8')
    if not suppressoutput and (len(out) > 0):
        print(out)
    nsp.wait()
    return out, err, nsp.returncode
    
def generate_privatekey():
    """Generates a WireGuard private key"""
    out, err, returncode = execute('wg genkey', suppressoutput=True)
    if (returncode != 0) or (len(err) > 0):
        return None
    out = out.strip() # remove trailing newline
    return out
    
def get_publickey(wg_private):
    """Gets the public key belonging to the given WireGuard private key"""
    if wg_private is None:
        return None
    out, err, returncode = execute('wg pubkey', input=wg_private, suppressoutput=True)
    if (returncode != 0) or (len(err) > 0):
        return None
    out = out.strip() # remove trailing newline
    return out
