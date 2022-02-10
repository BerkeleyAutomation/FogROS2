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


# This process pretends to do some simple setup, then pretends to do some simple work,
# then shuts itself down automatically
if __name__ == '__main__':

    print('Starting Up')
    time.sleep(1.0)
    print('Ready')

    if sys.argv[1:]:
        print('Called with arguments {}'.format(sys.argv[1:]))

    if '--exception' in sys.argv[1:]:
        raise Exception('Process had a pretend error')

    try:
        print('Emulating Work')
        time.sleep(1.0)
        print('Done')
    except KeyboardInterrupt:
        pass

    print('Shutting Down')

    sys.exit(0)
