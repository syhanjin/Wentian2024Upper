# -*- coding: utf-8 -*-
import os
from enum import Enum
import time
import random
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from std_msgs.msg import UInt8MultiArray

import asyncio
from pynput.keyboard import KeyCode, Key

from config import command_length
from utils.console import Widget, disable_echo
from utils.keyboard import Keyboard
from utils.color import *


class ChassisMotionStateChoice(int, Enum):
    Ready = 0x00
    Forward = 0x01
    Backward = 0x02
    Left = 0x04
    Right = 0x08


class ClawMotionStateChoice(int, Enum):
    Ready = 0x00
    Left = 0x01
    Right = 0x02
    Up = 0x04
    Down = 0x08
    Running = 0x10


def comparable_key(k: Key | KeyCode):
    return k.value if isinstance(k, Key) else k.char


class Controller(Node):
    chassis_speed_up_timer: Timer = None
    chassis_speed_down_timer: Timer = None
    claw_speed_up_timer: Timer = None
    claw_speed_down_timer: Timer = None
    keyboard: Keyboard = None

    widgets: dict[str, Widget] = {}

    def __init__(self):
        super().__init__('controller')
        self.command_publisher = self.create_publisher(UInt8MultiArray, 'command', command_length)
        self.data_subscriber = self.create_subscription(UInt8MultiArray, 'data', self.data_subscriber_callback,
                                                        command_length)
        self.init_widget()
        self.init_keyboard()

        self.start_state_send()

    def init_lower(self) -> None:
        """
        初始化下位机
        向下位机发送初始化信号，并等待回复
        :return: None
        """
        self.send([self._BEGIN, self._COMMANDS.RESET, *[random.randint(0, 255) for i in range(4)], self._END])

    def init_widget(self):
        """
        初始化控制台组件
        :return: None
        """
        self.widgets['ChassisForward'] = Widget(6, 2, yellow("{}"))
        self.widgets['ChassisBackward'] = Widget(6, 4, yellow("{}"))
        self.widgets['ChassisLeft'] = Widget(0, 3, yellow("{}"))
        self.widgets['ChassisRight'] = Widget(14, 3, yellow("{}"))
        self.widgets['ChassisReady'] = Widget(7, 3, yellow("Ready"), [])
        self.widgets['ChassisSpeed'] = Widget(5, 1, blue("Speed: {: >3d}"), [self.chassis_speed])
        self.widgets['ChassisTitle'] = Widget(4, 0, blue("- Chassis  -"), [])

        self.widgets['ClawUp'] = Widget(22 + 7, 2, yellow("{}"))
        self.widgets['ClawDown'] = Widget(22 + 6, 4, yellow("{}"))
        self.widgets['ClawLeft'] = Widget(22 + 0, 3, yellow("{}"))
        self.widgets['ClawRight'] = Widget(22 + 11, 3, yellow("{}"))
        self.widgets['ClawReady'] = Widget(22 + 5, 3, yellow("Ready"), [])
        self.widgets['ClawSpeed'] = Widget(22 + 3, 1, blue("Speed: {: >3d}"), [self.claw_speed])
        self.widgets['ClawTitle'] = Widget(22 + 2, 0, blue("-   Claw   -"), [])

    def init_keyboard(self):
        """
        初始化键盘，绑定快捷键
        :return: None
        """
        self.keyboard = Keyboard()

        self.keyboard.start_listening()
        self.keyboard.exit = self.exit
        # 底盘状态控制
        press, release = self.chassis_callback_wrapper()
        self.keyboard.key_press_callback['w'] = press
        self.keyboard.key_press_callback['a'] = press
        self.keyboard.key_press_callback['s'] = press
        self.keyboard.key_press_callback['d'] = press
        self.keyboard.key_release_callback['w'] = release
        self.keyboard.key_release_callback['a'] = release
        self.keyboard.key_release_callback['s'] = release
        self.keyboard.key_release_callback['d'] = release

        # 夹爪状态控制
        press, release = self.claw_callback_wrapper()
        self.keyboard.key_press_callback[Key.up.value] = press
        self.keyboard.key_press_callback[Key.left.value] = press
        self.keyboard.key_press_callback[Key.down.value] = press
        self.keyboard.key_press_callback[Key.right.value] = press
        self.keyboard.key_release_callback[Key.up.value] = release
        self.keyboard.key_release_callback[Key.left.value] = release
        self.keyboard.key_release_callback[Key.down.value] = release
        self.keyboard.key_release_callback[Key.right.value] = release

        # 底盘运动速度
        def start_ch_speed_up(*args, **kwargs):
            if self.chassis_speed_up_timer:
                return
            self.chassis_speed_up()
            self.chassis_speed_up_timer = self.create_timer(0.1, self.chassis_speed_up, )

        def stop_ch_speed_up(*args, **kwargs):
            if self.chassis_speed_up_timer:
                self.chassis_speed_up_timer.cancel()
                self.chassis_speed_up_timer = None

        def start_ch_speed_down(*args, **kwargs):
            if self.chassis_speed_down_timer:
                return
            self.chassis_speed_down()
            self.chassis_speed_down_timer = self.create_timer(0.1, self.chassis_speed_down, )

        def stop_ch_speed_down(*args, **kwargs):
            if self.chassis_speed_down_timer:
                self.chassis_speed_down_timer.cancel()
                self.chassis_speed_down_timer = None

        self.keyboard.key_press_callback['r'] = start_ch_speed_up
        self.keyboard.key_release_callback['r'] = stop_ch_speed_up
        self.keyboard.key_press_callback['f'] = start_ch_speed_down
        self.keyboard.key_release_callback['f'] = stop_ch_speed_down

        def ch_speed_to_top(*args, **kwargs):
            self.chassis_speed_up(True)

        def ch_speed_to_bottom(*args, **kwargs):
            self.chassis_speed_down(True)

        self.keyboard.key_release_callback['t'] = ch_speed_to_top
        self.keyboard.key_release_callback['g'] = ch_speed_to_bottom

        # 夹爪运动速度
        def start_cl_speed_up(*args, **kwargs):
            if self.claw_speed_up_timer:
                return
            self.claw_speed_up()
            self.claw_speed_up_timer = self.create_timer(0.1, self.claw_speed_up, )

        def stop_cl_speed_up(*args, **kwargs):
            if self.claw_speed_up_timer:
                self.claw_speed_up_timer.cancel()
                self.claw_speed_up_timer = None

        def start_cl_speed_down(*args, **kwargs):
            if self.claw_speed_down_timer:
                return
            self.claw_speed_down()
            self.claw_speed_down_timer = self.create_timer(0.1, self.claw_speed_down, )

        def stop_cl_speed_down(*args, **kwargs):
            if self.claw_speed_down_timer:
                self.claw_speed_down_timer.cancel()
                self.claw_speed_down_timer = None

        self.keyboard.key_press_callback["'"] = start_cl_speed_up
        self.keyboard.key_release_callback["'"] = stop_cl_speed_up
        self.keyboard.key_press_callback['/'] = start_cl_speed_down
        self.keyboard.key_release_callback['/'] = stop_cl_speed_down

    def exit(self):
        self.stop_state_send()
        self.destroy_node()
        rclpy.shutdown()

    def data_subscriber_callback(self, data):
        print(data.data)

    chassis_states: int = 0
    chassis_speed: int = 255

    claw_states: int = 0
    claw_speed: int = 255

    _BEGIN: int = 0xAA
    _END: int = 0xBB

    def state_add(self, chassis_state=None, claw_state=None):
        if chassis_state:
            self.chassis_states |= chassis_state
        if claw_state:
            self.claw_states |= claw_state

    def state_remove(self, chassis_state=None, claw_state=None):
        if chassis_state:
            self.chassis_states &= 0xFF - chassis_state
        if claw_state:
            self.claw_states &= 0xFF - claw_state

    class _COMMANDS(int, Enum):
        RESET = 0x03
        STATE = 0x05

    state_send_worker: Thread
    is_sending_state: bool = False

    def state_send_thread(self):
        while self.is_sending_state:
            self.send([self._BEGIN,
                       self._COMMANDS.STATE,
                       self.chassis_states,
                       self.chassis_speed,
                       self.claw_states,
                       self.claw_speed,
                       self._END, ])
            time.sleep(0.08)

    def start_state_send(self):
        self.is_sending_state = True
        self.state_send_worker = Thread(target=self.state_send_thread, daemon=True)
        self.state_send_worker.start()
        # print(green("State Send Worker Started."))

    def stop_state_send(self):
        self.is_sending_state = False
        self.state_send_worker.join()
        # print(green("State Send Worker Stop."))

    def send(self, data):
        """
        发送指令 指令长度 command_length=7
        :param data: 
        :return: 
        """
        msg = UInt8MultiArray(data=data)
        self.command_publisher.publish(msg)

    def chassis_speed_up(self, top=False):
        if self.chassis_speed == 255:
            return
        if top:
            self.chassis_speed = 255
        else:
            self.chassis_speed = min(self.chassis_speed + 5, 255)
        self.widgets['ChassisSpeed'].values = [self.chassis_speed]

    def chassis_speed_down(self, bottom=False):
        if self.chassis_speed == 1:
            return
        if bottom:
            self.chassis_speed = 1
        else:
            self.chassis_speed = max(self.chassis_speed - 5, 1)
        self.widgets['ChassisSpeed'].values = [self.chassis_speed]

    def claw_speed_up(self, top=False):
        if self.claw_speed == 255:
            return
        if top:
            self.claw_speed = 255
        else:
            self.claw_speed = min(self.claw_speed + 5, 255)
        self.widgets['ClawSpeed'].values = [self.claw_speed]

    def claw_speed_down(self, bottom=False):
        if self.claw_speed == 1:
            return
        if bottom:
            self.claw_speed = 1
        else:
            self.claw_speed = max(self.claw_speed - 5, 1)
        self.widgets['ClawSpeed'].values = [self.claw_speed]

    def chassis_callback_wrapper(self):

        def callback(key, is_add):
            key = comparable_key(key)
            if key == 'w':
                if is_add:
                    self.widgets['ChassisForward'].values = ['Forward']
                else:
                    self.widgets['ChassisForward'].values = ['']
                return ChassisMotionStateChoice.Forward
            elif key == 's':
                if is_add:
                    self.widgets['ChassisBackward'].values = ['Backward']
                else:
                    self.widgets['ChassisBackward'].values = ['']
                return ChassisMotionStateChoice.Backward
            elif key == 'a':
                if is_add:
                    self.widgets['ChassisLeft'].values = ['Left']
                else:
                    self.widgets['ChassisLeft'].values = ['']
                return ChassisMotionStateChoice.Left
            elif key == 'd':
                if is_add:
                    self.widgets['ChassisRight'].values = ['Right']
                else:
                    self.widgets['ChassisRight'].values = ['']
                return ChassisMotionStateChoice.Right
            return ChassisMotionStateChoice.Ready

        def press_callback(key: Key):
            self.state_add(chassis_state=callback(key, True))

        def release_callback(key: Key):
            self.state_remove(chassis_state=callback(key, False))

        return press_callback, release_callback

    def claw_callback_wrapper(self):
        def callback(key, is_add):
            key = comparable_key(key)
            if key == Key.up.value:
                if is_add:
                    self.widgets['ClawUp'].values = ['Up']
                else:
                    self.widgets['ClawUp'].values = ['']
                return ClawMotionStateChoice.Up
            elif key == Key.down.value:
                if is_add:
                    self.widgets['ClawDown'].values = ['Down']
                else:
                    self.widgets['ClawDown'].values = ['']
                return ClawMotionStateChoice.Down
            elif key == Key.left.value:
                if is_add:
                    self.widgets['ClawLeft'].values = ['Left']
                else:
                    self.widgets['ClawLeft'].values = ['']
                return ClawMotionStateChoice.Left
            elif key == Key.right.value:
                if is_add:
                    self.widgets['ClawRight'].values = ['Right']
                else:
                    self.widgets['ClawRight'].values = ['']
                return ClawMotionStateChoice.Right
            return ClawMotionStateChoice.Ready

        def press_callback(key: Key):
            self.state_add(claw_state=callback(key, True))

        def release_callback(key: Key):
            self.state_remove(claw_state=callback(key, False))

        return press_callback, release_callback


if __name__ == '__main__':
    with disable_echo():
        rclpy.init()
        os.system('clear')
        controller = Controller()
        rclpy.spin(controller)
        controller.destroy_node()
        # rclpy.shutdown() # 只有在按下ESC时才会返回，此时上下文已关闭
