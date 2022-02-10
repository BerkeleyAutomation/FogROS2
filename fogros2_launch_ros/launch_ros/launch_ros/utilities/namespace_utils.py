# Copyright 2020 Open Source Robotics Foundation, Inc.
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

"""Module with utility functions for handling namespaces."""


from typing import Optional
from typing import Text
from typing import Type
from typing import TypeVar


def is_root_namespace(ns: Text) -> bool:
    """Return `True` if `ns` is `'/'`."""
    return ns == '/'


def is_namespace_absolute(ns: Text) -> bool:
    """Return `True` if `ns` is absolute."""
    return ns.startswith('/')


def prefix_namespace(
    base_ns: Optional[Text],
    ns: Optional[Text],
) -> Optional[Text]:
    """
    Return `ns` prefixed with `base_ns` if `ns` is relative, return `ns` if not.

    :param `base_ns`: Prefix to be added to `ns`.
    :param `ns`: Namespace to be prefixed.
    :return: `None` if both `base_ns` and `ns` are `None`, or
        `base_ns` if `ns` is `None`, or
        `ns` if `base_ns` is `None`, or
        `ns` if `ns` is absolute, or
        `ns` prefixed with `base_ns`.
        In all cases, trailing `/` are stripped from the result.

    ## Examples:

    ```python3
    combined_ns = prefix_namespace('my_ns', 'original_ns')
    assert combined_ns == 'my_ns/original_ns'

    combined_ns = prefix_namespace('/my_ns', 'original_ns')
    assert combined_ns == '/my_ns/original_ns'

    combined_ns = prefix_namespace('my_ns', '/original_ns')
    assert combined_ns == '/original_ns'

    combined_ns = prefix_namespace(None, 'original_ns')
    assert combined_ns == 'original_ns'

    combined_ns = prefix_namespace('my_ns', None)
    assert combined_ns == 'my_ns'
    ```
    """
    combined_ns: Optional[Text] = None
    if base_ns is not None or ns is not None:
        if ns is None:
            combined_ns = base_ns
        elif not base_ns or is_namespace_absolute(ns):
            combined_ns = ns
        else:
            if is_root_namespace(base_ns):
                base_ns = ''
            combined_ns = (
                base_ns + '/' + ns
            )
        if not is_root_namespace(combined_ns):
            combined_ns = combined_ns.rstrip('/')
    return combined_ns


OptionalText = TypeVar('OptionalText', Text, Type[None])


def make_namespace_absolute(ns: OptionalText) -> OptionalText:
    """Make a relative namespace absolute."""
    if ns is None:
        return None
    if not is_namespace_absolute(ns):
        return '/' + ns
    return ns
