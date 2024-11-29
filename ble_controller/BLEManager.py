# -*- coding: utf-8 -*-
import termios
import tty
from argparse import ArgumentParser

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import asyncio
import sys
import os
from threading import Thread

from config import command_length
from utils.ble import Servers, ble_init
from utils.color import *

parser = ArgumentParser()

parser_scanner_group = parser.add_mutually_exclusive_group()
parser_scanner_group.add_argument('-a', '--address', type=str, default=None, help='设备mac地址')
parser_scanner_group.add_argument('-n', '--name', type=str, default=None, help='设备名称')

debug = False


class BLEManager(Node):
    _servers: Servers = None

    def __init__(self, name=None, address=None):
        super().__init__('ble_manager')
        if not debug:
            # 使用单独的线程启动事件循环
            self.loop = asyncio.new_event_loop()
            self.thread = Thread(target=self.start_loop, args=(self.loop,), daemon=True)
            self.thread.start()

            asyncio.run_coroutine_threadsafe(self.ble_init(name, address), self.loop)
        while not self._servers:
            pass
        print(blue("Creating Publisher and Subscription"))
        self.publisher = self.create_publisher(UInt8MultiArray, 'data', command_length)
        self.subscription = self.create_subscription(UInt8MultiArray, 'command', self.send_command, command_length)
        print(green("BLE Manager Is Ready."))

    def start_loop(self, loop):
        asyncio.set_event_loop(loop)
        loop.run_forever()

    async def ble_init(self, name, address):
        servers = await ble_init(name, address)
        assert servers is not None, red('连接设备失败')
        assert servers.main is not None, red('主服务通道不存在')
        assert servers.write is not None, red('写服务通道不存在')
        assert servers.read is not None, red('读服务通道不存在')
        self._servers = servers
        await self._servers.client.start_notify(self._servers.read, self.get_receive_func())

    def get_receive_func(self):
        def func(characteristic, data):
            if data[-1] != sum(data[:-1]) & 0xFF:
                return
            msg = UInt8MultiArray()
            msg.data = data[:-1]
            self.publisher.publish(msg)

        return func

    def send_command(self, msg):
        os.system('clear')
        data = msg.data
        print(data)
        data.append(sum(data) & 0xFF)
        print(blue(data))
        if debug:
            return
        if self._servers and self._servers.client.is_connected:
            asyncio.run_coroutine_threadsafe(
                self._servers.client.write_gatt_char(self._servers.write, bytearray(data), False),
                self.loop
            )

    def destroy_node(self):
        self._servers.client.disconnect()
        super().destroy_node()


def main():
    args = parser.parse_args(sys.argv[1:])
    rclpy.init()
    ble_manager = BLEManager(args.name, args.address)
    rclpy.spin(ble_manager)
    ble_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
