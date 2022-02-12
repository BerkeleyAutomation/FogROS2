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

import functools


def _normalize_to_tuple(val):
    if isinstance(val, tuple):
        return val
    else:
        return (val,)


def parametrize(argnames, argvalues):
    """
    Decorate a test launch description in a way that causes it to run with specified parameters.

    This decorator behaves like the @pytest.mark.parametrize decorator.

    :param: argnames A comma separated list of argument names.

    :param: argvalues The values to use for arguments specified in argnames
    """
    argnames = [x.strip() for x in argnames.split(',') if x.strip()]
    argvalues = [_normalize_to_tuple(x) for x in argvalues]

    def _decorator(func):
        @functools.wraps(func)
        def _wrapped():
            for val in argvalues:
                partial_args = dict(zip(argnames, val))

                partial = functools.partial(func, **partial_args)
                functools.update_wrapper(partial, func)
                yield partial, partial_args
        _wrapped.__parametrized__ = True
        return _wrapped

    return _decorator
