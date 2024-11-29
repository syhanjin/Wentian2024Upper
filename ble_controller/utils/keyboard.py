# keyboard.py
# -*- coding: utf-8 -*-
from threading import Thread
from types import FunctionType
from typing import Callable

from pynput import keyboard as pynput_keyboard

from .color import *


class Keyboard(object):
    _listener: pynput_keyboard.Listener
    _worker: Thread
    exit: Callable = None

    key_press_callback: dict[str | int, Callable] = {}
    key_release_callback: dict[str | int, Callable] = {}

    def __init__(self):
        self._listener = pynput_keyboard.Listener(on_press=self.on_press, on_release=self.on_release)

    def on_press(self, key: pynput_keyboard.Key | pynput_keyboard.KeyCode):
        # print(key, type(key))
        if key == pynput_keyboard.Key.esc:
            print(yellow('Got Esc, Exited.'))
            if self.exit:
                self.exit()
            self.stop_listening()
        if isinstance(key, pynput_keyboard.Key):
            callback = self.key_press_callback.get(key.value, None)
        else:
            callback = self.key_press_callback.get(key.char, None)
        if callback:
            try:
                callback(key)
            except Exception as e:
                pass

    def on_release(self, key: pynput_keyboard.Key | pynput_keyboard.KeyCode):
        # print(key, type(key))
        if isinstance(key, pynput_keyboard.Key):
            callback = self.key_release_callback.get(key.value, None)
        else:
            callback = self.key_release_callback.get(key.char, None)
        if callback:
            try:
                callback(key)
            except Exception as e:
                pass

    def start_listening(self):
        self._listener.start()

    def stop_listening(self):
        self._listener.stop()

    def is_listening(self):
        return self._listener.is_alive()
