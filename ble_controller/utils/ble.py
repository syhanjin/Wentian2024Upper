# ble.py
# -*- coding: utf-8 -*-

from bleak import BLEDevice, BleakClient, BleakGATTCharacteristic, BleakScanner
from bleak.backends.service import BleakGATTService
from bleak.uuids import normalize_uuid_16

from .cleanup import register_cleanup
from .color import *


async def scanner(name=None, address=None):
    if name:
        devices = await BleakScanner.find_device_by_name(name)
    elif address:
        devices = await BleakScanner.find_device_by_address(address)
    else:
        devices = await BleakScanner.discover()
    if isinstance(devices, BLEDevice):
        return [devices]
    return devices


class Servers:
    client: BleakClient
    write: BleakGATTCharacteristic
    read: BleakGATTCharacteristic
    main: BleakGATTService

    def __init__(self, client: BleakClient, main_addr=0xFFF0, read_addr=0xFFF1, write_addr=0xFFF2):
        self.client = client
        self.main = client.services.get_service(normalize_uuid_16(main_addr))
        assert self.main, red('Main service not found.')
        self.write = self.main.get_characteristic(normalize_uuid_16(write_addr))
        assert self.write, red('Write characteristic not found.')
        self.read = self.main.get_characteristic(normalize_uuid_16(read_addr))
        assert self.read, red('Read characteristic not found.')


async def ble_disconnect(client: BleakClient):
    print(blue("Disconnecting..."))
    await client.disconnect()
    print(green("Disconnected!"))


def exit_disconnect_wrapper(client: BleakClient):
    async def _disconnect_ble():
        await ble_disconnect(client)

    return _disconnect_ble


async def ble_init(name, address) -> Servers | None:
    print(red("Finding Devices..."))
    devices = await scanner(name, address)
    for index, device in enumerate(devices):
        print(f"{index}: {blue(device)}")
    device_id = -1 if len(devices) > 1 else 0
    while not 0 <= device_id < len(devices):
        try:
            device_id = int(input(green("Choose Device (ID): ")))
        except ValueError:
            print(red("NaN!"))
    device = devices[device_id]
    print(blue(f"Connecting({device})..."))
    ble_client = BleakClient(device)
    await ble_client.connect()
    print(green("Connected!"))
    register_cleanup(exit_disconnect_wrapper(ble_client))
    print(blue("Getting Service..."))
    try:
        servers = Servers(ble_client)
        print(green("Services Found."))
        return servers
    except AssertionError as e:
        print(e)
        return None
