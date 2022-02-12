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

"""Module for the launch specific logging."""

import codecs
import datetime
import locale
import logging
import logging.handlers

import os
import socket
import sys

from typing import Iterable
from typing import List

from . import handlers

from ..frontend import expose_substitution
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitutions import TextSubstitution

__all__ = [
    'get_logger',
    'get_output_loggers',
    'handlers',
    'launch_config',
    'reset',
]


def _get_logging_directory():
    """
    Get logging directory path.

    Uses various environment variables to construct a logging directory path.

    Use $ROS_LOG_DIR if ROS_LOG_DIR is set and not empty.
    Otherwise, use $ROS_HOME/log, using ~/.ros for ROS_HOME if not set or if empty.

    It also expands '~' to the current user's home directory,
    and normalizes the path, converting the path separator if necessary.

    :return: the path to the logging directory
    """
    log_dir = os.environ.get('ROS_LOG_DIR')
    if not log_dir:
        log_dir = os.environ.get('ROS_HOME')
        if not log_dir:
            log_dir = os.path.join('~', '.ros')
        log_dir = os.path.join(log_dir, 'log')
    return os.path.normpath(os.path.expanduser(log_dir))


def _make_unique_log_dir(*, base_path):
    """
    Make a unique directory for logging.

    :param: base_path for directory creation
    :return: the path to the created directory
    """
    while True:
        now = datetime.datetime.now()
        datetime_str = now.strftime('%Y-%m-%d-%H-%M-%S-%f')
        log_dirname = '{0}-{1}-{2}'.format(
            datetime_str, socket.gethostname(), os.getpid()
        )
        log_dir = os.path.join(base_path, log_dirname)
        # Check that filename does not exist
        # TODO(hidmic): fix (unlikely) TOCTTOU race
        if not os.path.isdir(log_dir):
            os.makedirs(log_dir, exist_ok=True)
            return log_dir


class LaunchConfig:
    """Launch Logging Configuration class."""

    def __init__(self):
        self.reset()

    def reset(self):
        self._log_dir = None
        self.file_handlers = {}
        self.screen_handler = None
        self.screen_formatter = None
        self.file_formatter = None
        self._log_handler_factory = None
        logging.root.setLevel(logging.INFO)
        self.set_screen_format('default')
        self.set_log_format('default')

    @property
    def level(self):
        return logging.root.getEffectiveLevel()

    @level.setter
    def level(self, new_level):
        """
        Set up launch logging verbosity level for all loggers.

        :param new_level: the default log level used for all loggers.
        """
        logging.root.setLevel(new_level)

    @property
    def log_dir(self):
        """Get the current log directory, generating it if necessary."""
        if self._log_dir is None:
            self._log_dir = _make_unique_log_dir(
                base_path=_get_logging_directory()
            )

        return self._log_dir

    @log_dir.setter
    def log_dir(self, new_log_dir):
        """
        Set up launch logging directory.

        :param new_log_dir: used as base path for all log file collections.
        """
        if new_log_dir is not None:
            if any(self.file_handlers):
                import warnings
                warnings.warn((
                    'Loggers have been already configured to output to log files below {}. '
                    'Proceed at your own risk.'
                ).format(self._log_dir))
            if not os.path.isdir(new_log_dir):
                raise ValueError('{} is not a directory'.format(new_log_dir))
        self._log_dir = new_log_dir

    @property
    def log_handler_factory(self):
        """Get the log_handler_factory, generating it if necessary."""
        if self._log_handler_factory is None:
            if os.name != 'nt':
                self._log_handler_factory = handlers.WatchedFileHandler
            else:
                self._log_handler_factory = handlers.FileHandler
        return self._log_handler_factory

    @log_handler_factory.setter
    def log_handler_factory(self, new_log_handler_factory):
        """
        Set up log handler factory.

        :param new_log_handler_factory: a callable to build log file handler.
           It takes a path to the log file and it must return a `logging.Handler`
           variant with per logger formatting support.
           See `launch.logging.handlers` module for further reference and easy reuse
           of existing standard `logging.handlers` module handlers.
           Defaults to regular log file handlers for logging if no factory is given.
        """
        self._log_handler_factory = new_log_handler_factory

    def set_screen_format(self, screen_format, *, screen_style=None):
        """
        Set up screen formats.

        For the ``screen_format`` argument there are a few aliases:

          - 'default' to log verbosity level, logger name and logged message
          - 'default_with_timestamp' to add timestamps to the 'default' format

        :param screen_format: format specification used when logging to the screen,
            as expected by the `logging.Formatter` constructor.
            Alternatively, aliases for common formats are available, see above.
        :param screen_style: the screen style used if no alias is used for
            screen_format.
            No style can be provided if a format alias is given.
        """
        if screen_format is not None:
            if screen_format == 'default':
                screen_format = '[{levelname}] [{name}]: {msg}'
                if screen_style is not None:
                    raise ValueError(
                        'Cannot set a custom format style for the "default" screen format.'
                    )
            if screen_format == 'default_with_timestamp':
                screen_format = '{created:.7f} [{levelname}] [{name}]: {msg}'
                if screen_style is not None:
                    raise ValueError(
                        'Cannot set a custom format style for the '
                        '"default_with_timestamp" screen format.'
                    )
            if screen_style is None:
                screen_style = '{'
            self.screen_formatter = logging.Formatter(
                screen_format, style=screen_style
            )
            if self.screen_handler is not None:
                self.screen_handler.setFormatter(self.screen_formatter)
        else:
            self.screen_formatter = None

    def get_screen_handler(self):
        """
        Get the one and only screen logging handler.

        See launch_config() documentation for screen logging configuration.
        """
        if self.screen_handler is None:
            stream = codecs.StreamWriter(sys.stdout, errors='replace')
            stream.encode = lambda msg, errors='replace': (
                msg.encode(locale.getpreferredencoding(False), errors).decode(
                    locale.getpreferredencoding(False), errors=errors),
                msg)
            self.screen_handler = handlers.StreamHandler(stream)
            self.screen_handler.setFormatter(self.screen_formatter)
        return self.screen_handler

    def set_log_format(self, log_format, *, log_style=None):
        """
        Set up launch log file format.

        :param log_format: the format used when logging to the main launch log file,
            as expected by the `logging.Formatter` constructor.
            Alternatively, the 'default' alias can be given to log verbosity level,
            logger name and logged message.
        :param log_style: the log style used if no alias is given for log_format.
            No style can be provided if a format alias is given.
        """
        if log_format is not None:
            if log_format == 'default':
                log_format = '{created:.7f} [{levelname}] [{name}]: {msg}'
                if log_style is not None:
                    raise ValueError(
                        'Cannot set a custom format style for the "default" log format.'
                    )
            if log_style is None:
                log_style = '{'
            self.file_formatter = logging.Formatter(
                log_format, style=log_style
            )
            for handler in self.file_handlers.values():
                handler.setFormatter(self.file_formatter)
        else:
            self.file_formatter = None

    def get_log_file_path(self, file_name='launch.log'):
        """
        Get the absolute path to the given log file.

        :param: file_name of the log file from which to get the absolute path.
        :return: the absolute path to the log file.
        """
        return os.path.join(self.log_dir, file_name)

    def get_log_file_handler(self, file_name='launch.log'):
        """
        Get the logging handler to a log file.

        See launch_config() documentation for application wide log file
        logging configuration.

        :param: file_name of the log file whose handler is to be retrieved.
        :return: the logging handler associated to the file (always the same
        once constructed).
        """
        if file_name not in self.file_handlers:
            file_path = self.get_log_file_path(file_name)
            factory = self.log_handler_factory
            file_handler = factory(file_path, encoding='utf-8')
            file_handler.setFormatter(self.file_formatter)
            self.file_handlers[file_name] = file_handler
        return self.file_handlers[file_name]


launch_config = LaunchConfig()


def log_launch_config(*, logger=logging.root):
    """Log logging configuration details relevant for a user with the given logger."""
    if any(launch_config.file_handlers):
        logger.info('All log files can be found below {}'.format(launch_config.log_dir))
    logger.info('Default logging verbosity is set to {}'.format(logging.getLevelName(
        logging.root.getEffectiveLevel()
    )))


def get_logger(name=None):
    """Get named logger, configured to output to screen and launch main log file."""
    logger = logging.getLogger(name)
    screen_handler = launch_config.get_screen_handler()
    if screen_handler not in logger.handlers:
        logger.addHandler(screen_handler)
    launch_log_file_handler = launch_config.get_log_file_handler()
    if launch_log_file_handler not in logger.handlers:
        logger.addHandler(launch_log_file_handler)
    return logger


@expose_substitution('log_dir')
def _log_dir(data: Iterable[SomeSubstitutionsType]):
    if len(data) > 0:
        raise ValueError('log_dir substitution does not expect any arguments')
    return TextSubstitution, {'text': launch_config.log_dir}


def _normalize_output_configuration(config):
    """
    Normalize output configuration to a dict representation.

    See `get_output_loggers()` documentation for further reference.
    """
    normalized_config = {
        'both': set(), 'stdout': set(), 'stderr': set()
    }
    if isinstance(config, str):
        if config == 'screen':
            normalized_config.update({
                'both': {'screen'}
            })
        elif config == 'log':
            normalized_config.update({
                'both': {'log'},
                'stderr': {'screen'}
            })
        elif config == 'both':
            normalized_config.update({
                'both': {'log', 'screen'},
            })
        elif config == 'own_log':
            normalized_config.update({
                'both': {'own_log'},
                'stdout': {'own_log'},
                'stderr': {'own_log'}
            })
        elif config == 'full':
            normalized_config.update({
                'both': {'screen', 'log', 'own_log'},
                'stdout': {'own_log'},
                'stderr': {'own_log'}
            })
        else:
            raise ValueError((
                '{} is not a valid standard output config '
                'i.e. "screen", "log" or "both"'
            ).format(config))
    elif isinstance(config, dict):
        for source, destinations in config.items():
            if source not in ('stdout', 'stderr', 'both'):
                raise ValueError((
                    '{} is not a valid output source '
                    'i.e. "stdout", "stderr" or "both"'
                ).format(source))
            if isinstance(destinations, str):
                destinations = {destinations}
            for destination in destinations:
                if destination not in ('screen', 'log', 'own_log'):
                    raise ValueError((
                        '{} is not a valid output destination '
                        'i.e. "screen", "log" or "own_log"'
                    ).format(destination))
            normalized_config[source] = set(destinations)
    else:
        raise ValueError(
            '{} is not a valid output configuration'.format(config)
        )
    return normalized_config


def get_output_loggers(process_name, output_config):
    """
    Get the stdout and stderr output loggers for the given process name.

    The output_config may be a dictionary with one or more of the optional keys
    'stdout', 'stderr', or 'both' (stdout and stderr combined) which represent
    the various process output sources, and values for those keys to assign one
    or more logging destinations to the source.
    The logging destination values may be:

      - 'screen': log it to the screen,
      - 'log': log it to launch log file, or
      - 'own_log': log it to a separate log file.

    When logging the stdout and stderr separately, the log file names follow
    the ``<process_name>-<source>.log`` pattern where ``<source>`` is either
    'stdout' or 'stderr'
    When the 'both' logging destination is used the log file name follows the
    ``<process_name>.log`` pattern.

    The "launch log file" is a log file which is create for each run of
    the launch.LaunchService, and at least captures the log output from launch
    itself, but may also include output from subprocess's if configured so.

    Alternatively, the output_config parameter may be a string which represents
    one of a couple available aliases for common logging configurations.
    The available aliases are:

      - 'screen': stdout and stderr are logged to the screen,
      - 'log': stdout and stderr are logged to launch log file and stderr to
            the screen,
      - 'both': both stdout and stderr are logged to the screen and to launch
            main log file,
      - 'own_log' for stdout, stderr and their combination to be logged to
            their own log files, and
      - 'full' to have stdout and stderr sent to the screen, to the main launch
            log file, and their own separate and combined log files.

    :param process_name: the process-like action whose outputs want to be logged.
    :param output_config: configuration for the output loggers,
        see above for details.
    :returns: a tuple with the stdout and stderr output loggers.
    """
    output_config = _normalize_output_configuration(output_config)
    for source in ('stdout', 'stderr'):
        logger = logging.getLogger('{}-{}'.format(process_name, source))
        # If a 'screen' output is configured for this source or for
        # 'both' sources, this logger should output to screen.
        if 'screen' in (output_config['both'] | output_config[source]):
            screen_handler = launch_config.get_screen_handler()
            # Add screen handler if necessary.
            if screen_handler not in logger.handlers:
                screen_handler.setFormatterFor(
                    logger, logging.Formatter('{msg}', style='{')
                )
                logger.addHandler(screen_handler)

        # If a 'log' output is configured for this source or for
        # 'both' sources, this logger should output to launch main log file.
        if 'log' in (output_config['both'] | output_config[source]):
            launch_log_file_handler = launch_config.get_log_file_handler()
            # Add launch main log file handler if necessary.
            if launch_log_file_handler not in logger.handlers:
                launch_log_file_handler.setFormatterFor(
                    logger, logging.Formatter('{created:.7f} {msg}', style='{')
                )
                logger.addHandler(launch_log_file_handler)

        # If an 'own_log' output is configured for this source, this logger
        # should output to its own log file.
        if 'own_log' in output_config[source]:
            own_log_file_handler = launch_config.get_log_file_handler(
                '{}-{}.log'.format(process_name, source)
            )
            own_log_file_handler.setFormatter(logging.Formatter(fmt=None))
            # Add own log file handler if necessary.
            if own_log_file_handler not in logger.handlers:
                logger.addHandler(own_log_file_handler)
        # If an 'own_log' output is configured for 'both' sources,
        # this logger should output to a combined log file.
        if 'own_log' in output_config['both']:
            combined_log_file_handler = launch_config.get_log_file_handler(process_name + '.log')
            combined_log_file_handler.setFormatter(logging.Formatter('{msg}', style='{'))
            # Add combined log file handler if necessary.
            if combined_log_file_handler not in logger.handlers:
                logger.addHandler(combined_log_file_handler)
    # Retrieve both loggers.
    return (
        logging.getLogger(process_name + '-stdout'),
        logging.getLogger(process_name + '-stderr')
    )


# Track all loggers to support module resets
class LaunchLogger(logging.getLoggerClass()):
    all_loggers: List[logging.Logger] = []

    def __new__(cls, *args, **kwargs):
        instance = super(LaunchLogger, cls).__new__(cls)
        LaunchLogger.all_loggers.append(instance)
        return instance

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.propagate = False


def reset():
    """Reset logging."""
    # Reset existing logging infrastructure
    for logger in LaunchLogger.all_loggers:
        logger.setLevel(logging.NOTSET)
        del logger.handlers[:]
    launch_config.reset()
    logging.setLoggerClass(LaunchLogger)


# Initial module reset
reset()
