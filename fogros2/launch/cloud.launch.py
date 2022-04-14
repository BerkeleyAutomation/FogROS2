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

import pickle

from launch import LaunchDescription


def generate_launch_description():
    ld = LaunchDescription()
    node_path = "/tmp/to_cloud_nodes"
    # [os.path.join(node_dir, file) for file in os.listdir(node_dir) if file.startswith("to_cloud")]
    with open(node_path, "rb") as f:
        nodes_in_str = f.read()
    nodes = pickle.loads(nodes_in_str)
    for node in nodes:
        ld.add_action(node)
        print("action added")
    return ld
