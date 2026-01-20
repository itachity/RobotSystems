#!/usr/bin/env python3
import os
import multiprocessing

from .basic import _Basic_class
from .utils import run_command

# Try to import SMBus (works on Raspberry Pi / Linux)
# On Windows this will fail because smbus2 depends on fcntl.
try:
    from smbus2 import SMBus
except Exception:
    SMBus = None


def _retry_wrapper(func):
    def wrapper(self, *arg, **kwargs):
        for _ in range(self.RETRY):
            try:
                return func(self, *arg, **kwargs)
            except OSError:
                self._debug(f"OSError: {func.__name__}")
                continue
        else:
            return False

    return wrapper


class FakeSMBus:
    """
    Desktop/offline fake I2C bus (Windows-safe).
    All writes do nothing.
    Reads return zeros of the correct type.
    """
    def __init__(self, bus=1):
        self.bus = bus

    def write_byte(self, addr, data):
        return 0

    def write_byte_data(self, addr, reg, data):
        return 0

    def write_word_data(self, addr, reg, data):
        return 0

    def write_i2c_block_data(self, addr, reg, data):
        return 0

    def read_byte(self, addr):
        return 0

    def read_byte_data(self, addr, reg):
        return 0

    def read_word_data(self, addr, reg):
        return 0

    def read_i2c_block_data(self, addr, reg, num):
        return [0] * num

    def close(self):
        pass


class I2C(_Basic_class):
    """
    I2C bus read/write functions
    """
    RETRY = 5

    def __init__(self, address=None, bus=1, *args, **kwargs):
        """
        Initialize the I2C bus

        :param address: I2C device address
        :type address: int or list[int]
        :param bus: I2C bus number
        :type bus: int
        """
        super().__init__(*args, **kwargs)
        self._bus = bus

        # If Windows OR SMBus couldn't import, use FakeSMBus
        self._offline = (os.name == "nt") or (SMBus is None)
        self._smbus = FakeSMBus(self._bus) if self._offline else SMBus(self._bus)

        # On desktop, don't run scan() / i2cdetect
        if isinstance(address, list):
            if self._offline:
                self.address = address[0]
            else:
                connected_devices = self.scan()
                for _addr in address:
                    if _addr in connected_devices:
                        self.address = _addr
                        break
                else:
                    self.address = address[0]
        else:
            self.address = address

    @_retry_wrapper
    def _write_byte(self, data):
        self._debug(f"_write_byte: [0x{data:02X}]")
        return self._smbus.write_byte(self.address, data)

    @_retry_wrapper
    def _write_byte_data(self, reg, data):
        self._debug(f"_write_byte_data: [0x{reg:02X}] [0x{data:02X}]")
        return self._smbus.write_byte_data(self.address, reg, data)

    @_retry_wrapper
    def _write_word_data(self, reg, data):
        self._debug(f"_write_word_data: [0x{reg:02X}] [0x{data:04X}]")
        return self._smbus.write_word_data(self.address, reg, data)

    @_retry_wrapper
    def _write_i2c_block_data(self, reg, data):
        self._debug(
            f"_write_i2c_block_data: [0x{reg:02X}] {[f'0x{i:02X}' for i in data]}"
        )
        return self._smbus.write_i2c_block_data(self.address, reg, data)

    @_retry_wrapper
    def _read_byte(self):
        result = self._smbus.read_byte(self.address)
        self._debug(f"_read_byte: [0x{result:02X}]")
        return result

    @_retry_wrapper
    def _read_byte_data(self, reg):
        result = self._smbus.read_byte_data(self.address, reg)
        self._debug(f"_read_byte_data: [0x{reg:02X}] [0x{result:02X}]")
        return result

    @_retry_wrapper
    def _read_word_data(self, reg):
        result = self._smbus.read_word_data(self.address, reg)
        result_list = [result & 0xFF, (result >> 8) & 0xFF]
        self._debug(f"_read_word_data: [0x{reg:02X}] [0x{result:04X}]")
        return result_list

    @_retry_wrapper
    def _read_i2c_block_data(self, reg, num):
        result = self._smbus.read_i2c_block_data(self.address, reg, num)
        self._debug(
            f"_read_i2c_block_data: [0x{reg:02X}] {[f'0x{i:02X}' for i in result]}"
        )
        return result

    @_retry_wrapper
    def is_ready(self):
        # Offline: pretend device is ready so the rest of the stack can run
        if self._offline:
            return True

        addresses = self.scan()
        return self.address in addresses

    def scan(self):
        """
        Scan the I2C bus for devices
        """
        # Offline: no i2cdetect available on Windows
        if self._offline:
            return []

        cmd = f"i2cdetect -y {self._bus}"
        _, output = run_command(cmd)

        outputs = output.split('\n')[1:]
        addresses = []
        addresses_str = []

        for tmp_addresses in outputs:
            if tmp_addresses == "":
                continue
            tmp_addresses = tmp_addresses.split(':')[1]
            tmp_addresses = tmp_addresses.strip().split(' ')
            for address in tmp_addresses:
                if address != '--':
                    addresses.append(int(address, 16))
                    addresses_str.append(f'0x{address}')

        self._debug(f"Connected i2c device: {addresses_str}")
        return addresses

    def write(self, data):
        """
        Write data to the I2C device
        """
        if isinstance(data, bytearray):
            data_all = list(data)
        elif isinstance(data, int):
            if data == 0:
                data_all = [0]
            else:
                data_all = []
                while data > 0:
                    data_all.append(data & 0xFF)
                    data >>= 8
        elif isinstance(data, list):
            data_all = data
        else:
            raise ValueError(
                f"write data must be int, list, or bytearray, not {type(data)}"
            )

        if len(data_all) == 1:
            self._write_byte(data_all[0])
        elif len(data_all) == 2:
            self._write_byte_data(data_all[0], data_all[1])
        elif len(data_all) == 3:
            reg = data_all[0]
            data_word = (data_all[2] << 8) + data_all[1]
            self._write_word_data(reg, data_word)
        else:
            reg = data_all[0]
            self._write_i2c_block_data(reg, list(data_all[1:]))

    def read(self, length=1):
        """
        Read data from I2C device
        """
        if not isinstance(length, int):
            raise ValueError(f"length must be int, not {type(length)}")

        return [self._read_byte() for _ in range(length)]

    def mem_write(self, data, memaddr):
        if isinstance(data, bytearray):
            data_all = list(data)
        elif isinstance(data, list):
            data_all = data
        elif isinstance(data, int):
            if data == 0:
                data_all = [0]
            else:
                data_all = []
                while data > 0:
                    data_all.append(data & 0xFF)
                    data >>= 8
        else:
            raise ValueError("mem_write needs bytearray, list, or int")

        self._write_i2c_block_data(memaddr, data_all)

    def mem_read(self, length, memaddr):
        return self._read_i2c_block_data(memaddr, length)

    def is_avaliable(self):
        if self._offline:
            return True
        return self.address in self.scan()

    def __del__(self):
        try:
            if self._smbus is not None:
                self._smbus.close()
        except Exception:
            pass
        self._smbus = None


if __name__ == "__main__":
    i2c = I2C(address=[0x17, 0x15], debug_level='debug')
