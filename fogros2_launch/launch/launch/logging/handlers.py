# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Module with handlers for launch specific logging."""

import sys


def with_per_logger_formatting(cls):
    """Add per logger formatting capabilities to the given logging.Handler."""
    class _trait(cls):
        """A logging.Handler subclass to enable per logger formatting."""

        def __init__(self, *args, **kwargs):
            super(_trait, self).__init__(*args, **kwargs)
            self._formatters = {}

        def setFormatterFor(self, logger, formatter):
            """Set formatter for a given logger instance or logger name."""
            logger_name = logger if isinstance(logger, str) else logger.name
            self._formatters[logger_name] = formatter

        def unsetFormatterFor(self, logger):
            """Unset formatter for a given logger instance or logger name, if any."""
            logger_name = logger if isinstance(logger, str) else logger.name
            if logger_name in self._formatters:
                del self._formatters[logger_name]

        def format(self, record):  # noqa
            if record.name in self._formatters:
                formatter = self._formatters[record.name]
                return formatter.format(record)
            return super(_trait, self).format(record)

    _trait.__name__ = cls.__name__
    _trait.__doc__ = cls.__doc__
    return _trait


# TODO(hidmic): replace module wrapper with module-level __getattr__
#               implementation when we switch to Python 3.7+
class _module_wrapper:
    """Provide all Python `logging` module handlers with per logger formatting support."""

    def __init__(self, wrapped_module):
        import logging
        import logging.handlers
        self._handlers = {}
        for module in (logging, logging.handlers):
            for name in dir(module):
                if name.startswith('_'):
                    continue
                obj = getattr(module, name)
                if not isinstance(obj, type):
                    continue
                if not issubclass(obj, logging.Handler):
                    continue
                self._handlers[name] = with_per_logger_formatting(obj)
        self._module = module

    def __getattr__(self, name):
        if name in self._handlers:
            return self._handlers[name]
        return getattr(self._module, name)


sys.modules[__name__] = _module_wrapper(sys.modules[__name__])
