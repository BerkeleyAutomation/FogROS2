# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Module for signal management functionality."""

import asyncio
from contextlib import ExitStack
import os
import platform
import signal
import socket
import threading

from typing import Callable
from typing import Optional
from typing import Tuple  # noqa: F401
from typing import Union


class AsyncSafeSignalManager:
    """
    A context manager class for asynchronous handling of signals.

    Similar in purpose to :func:`asyncio.loop.add_signal_handler` but
    not limited to Unix platforms.

    Signal handlers can be registered at any time with a given manager.
    These will become active for the extent of said manager context.
    Unlike regular signal handlers, asynchronous signals handlers
    can safely interact with their event loop.

    The same manager can be used multiple consecutive times and even
    be nested with other managers, as these are independent from each
    other i.e. managers do not override each other's handlers.

    If used outside of the main thread, a ValueError is raised.

    The underlying mechanism is built around :func:`signal.set_wakeup_fd`
    so as to not interfere with regular handlers installed via
    :func:`signal.signal`.
    All signals received are forwarded to the previously setup file
    descriptor, if any.

    ..warning::
        Within (potentially nested) contexts, :func:`signal.set_wakeup_fd`
        calls are intercepted such that the given file descriptor overrides
        the previously setup file descriptor for the outermost manager.
        This ensures the manager's chain of signal wakeup file descriptors
        is not broken by third-party code or by asyncio itself in some platforms.
    """

    __current = None  # type: AsyncSafeSignalManager

    __set_wakeup_fd = signal.set_wakeup_fd  # type: Callable[[int], int]

    def __init__(
        self,
        loop: asyncio.AbstractEventLoop
    ):
        """
        Instantiate manager.

        :param loop: event loop that will handle the signals.
        """
        self.__parent = None  # type: AsyncSafeSignalManager
        self.__loop = loop  # type: asyncio.AbstractEventLoop
        self.__background_loop = None  # type: Optional[asyncio.AbstractEventLoop]
        self.__handlers = {}  # type: dict
        self.__prev_wakeup_handle = -1  # type: Union[int, socket.socket]
        self.__wsock = None
        self.__rsock = None
        self.__close_sockets = None

    def __enter__(self):
        pair = socket.socketpair()  # type: Tuple[socket.socket, socket.socket]  # noqa
        with ExitStack() as stack:
            self.__wsock = stack.enter_context(pair[0])
            self.__rsock = stack.enter_context(pair[1])
            self.__wsock.setblocking(False)
            self.__rsock.setblocking(False)
            self.__close_sockets = stack.pop_all().close

        self.__add_signal_readers()
        try:
            self.__install_signal_writers()
        except Exception:
            self.__remove_signal_readers()
            self.__close_sockets()
            self.__rsock = None
            self.__wsock = None
            self.__close_sockets = None
            raise
        self.__chain()
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        try:
            try:
                self.__uninstall_signal_writers()
            finally:
                self.__remove_signal_readers()
        finally:
            self.__unchain()
            self.__close_sockets()
            self.__rsock = None
            self.__wsock = None
            self.__close_sockets = None

    def __add_signal_readers(self):
        try:
            self.__loop.add_reader(self.__rsock.fileno(), self.__handle_signal)
        except NotImplementedError:
            # Some event loops, like the asyncio.ProactorEventLoop
            # on Windows, do not support asynchronous socket reads.
            # Emulate it.
            self.__background_loop = asyncio.SelectorEventLoop()
            self.__background_loop.add_reader(
                self.__rsock.fileno(),
                self.__loop.call_soon_threadsafe,
                self.__handle_signal)

            def run_background_loop():
                asyncio.set_event_loop(self.__background_loop)
                self.__background_loop.run_forever()

            self.__background_thread = threading.Thread(
                target=run_background_loop, daemon=True)
            self.__background_thread.start()

    def __remove_signal_readers(self):
        if self.__background_loop:
            self.__background_loop.call_soon_threadsafe(self.__background_loop.stop)
            self.__background_thread.join()
            self.__background_loop.close()
            self.__background_loop = None
        else:
            self.__loop.remove_reader(self.__rsock.fileno())

    def __install_signal_writers(self):
        prev_wakeup_handle = self.__set_wakeup_fd(self.__wsock.fileno())
        try:
            self.__chain_wakeup_handle(prev_wakeup_handle)
        except Exception:
            own_wakeup_handle = self.__set_wakeup_fd(prev_wakeup_handle)
            assert self.__wsock.fileno() == own_wakeup_handle
            raise

    def __uninstall_signal_writers(self):
        prev_wakeup_handle = self.__chain_wakeup_handle(-1)
        own_wakeup_handle = self.__set_wakeup_fd(prev_wakeup_handle)
        assert self.__wsock.fileno() == own_wakeup_handle

    def __chain(self):
        self.__parent = AsyncSafeSignalManager.__current
        AsyncSafeSignalManager.__current = self
        if self.__parent is None:
            # Do not trust signal.set_wakeup_fd calls within context.
            # Overwrite handle at the start of the managers' chain.
            def modified_set_wakeup_fd(signum):
                if threading.current_thread() is not threading.main_thread():
                    raise ValueError(
                        'set_wakeup_fd only works in main'
                        ' thread of the main interpreter'
                    )
                return self.__chain_wakeup_handle(signum)
            signal.set_wakeup_fd = modified_set_wakeup_fd

    def __unchain(self):
        if self.__parent is None:
            signal.set_wakeup_fd = self.__set_wakeup_fd
        AsyncSafeSignalManager.__current = self.__parent

    def __chain_wakeup_handle(self, wakeup_handle):
        prev_wakeup_handle = self.__prev_wakeup_handle
        if isinstance(prev_wakeup_handle, socket.socket):
            # Detach (Windows) socket and retrieve the raw OS handle.
            prev_wakeup_handle = prev_wakeup_handle.detach()
        if wakeup_handle != -1 and platform.system() == 'Windows':
            # On Windows, os.write will fail on a WinSock handle. There is no WinSock API
            # in the standard library either. Thus we wrap it in a socket.socket instance.
            try:
                wakeup_handle = socket.socket(fileno=wakeup_handle)
            except WindowsError as e:
                if e.winerror != 10038:  # WSAENOTSOCK
                    raise
        self.__prev_wakeup_handle = wakeup_handle
        return prev_wakeup_handle

    def __handle_signal(self):
        while True:
            try:
                data = self.__rsock.recv(4096)
                if not data:
                    break
                for signum in data:
                    if signum not in self.__handlers:
                        continue
                    self.__handlers[signum](signum)
                if self.__prev_wakeup_handle != -1:
                    # Send over (Windows) socket or write file.
                    if isinstance(self.__prev_wakeup_handle, socket.socket):
                        self.__prev_wakeup_handle.send(data)
                    else:
                        os.write(self.__prev_wakeup_handle, data)
            except InterruptedError:
                continue
            except BlockingIOError:
                break

    def handle(
        self,
        signum: Union[signal.Signals, int],
        handler: Optional[Callable[[int], None]],
    ) -> Optional[Callable[[int], None]]:
        """
        Register a callback for asynchronous handling of a given signal.

        :param signum: number of the signal to be handled
        :param handler: callback taking a signal number
          as its sole argument, or None
        :return: previous handler if any, otherwise None
        """
        signum = signal.Signals(signum)
        if handler is not None:
            if not callable(handler):
                raise ValueError('signal handler must be a callable')
            old_handler = self.__handlers.get(signum, None)
            self.__handlers[signum] = handler
        else:
            old_handler = self.__handlers.pop(signum, None)
        return old_handler
