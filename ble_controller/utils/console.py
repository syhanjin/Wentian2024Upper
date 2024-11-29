# -*- coding: utf-8 -*-
import os
import sys
import termios
import tty
from contextlib import contextmanager


def goto(x: int, y: int):
    print(f'\033[{y};{x}H')


class Widget:
    x: int = 0
    y: int = 0
    length: int = 0
    format: str = ""
    _values = []

    @property
    def pos(self):
        return self.x, self.y

    @pos.setter
    def pos(self, value):
        self.x, self.y = value

    @property
    def values(self):
        return self._values

    @values.setter
    def values(self, value):
        self._values = value
        self.display()

    def __init__(self, x, y, format_, values=None):
        self.x = x
        self.y = y
        self.format = format_
        if values is not None:
            self.values = values

    def display(self):
        goto(self.x, self.y)
        output = self.format.format(*self.values)
        print(output.ljust(self.length))
        self.length = len(output)


@contextmanager
def disable_echo():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    print("\033[?25l")
    try:
        tty.setraw(fd)
        yield
    finally:
        print("\033[?25h")
        termios.tcflush(fd, termios.TCIFLUSH)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        os.system('clear')
