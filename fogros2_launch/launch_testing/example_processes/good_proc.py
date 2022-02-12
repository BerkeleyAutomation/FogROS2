#!/usr/bin/env python3

# Copyright 2019 Apex.AI, Inc.
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

import sys
import time


# This is a simple program that generates some stdout, waits for ctrl+c, and exits with
# an exit code of zero
if __name__ == '__main__':

    if sys.argv[1:]:
        print('Called with arguments {}'.format(sys.argv[1:]))

    print('Starting Up')

    loops = 0
    try:
        while True:
            print('Loop {}'.format(loops))
            loops += 1
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

    print('Shutting Down')

    sys.exit(0)
