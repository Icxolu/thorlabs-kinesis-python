from MotionControl.KinesisTypes import *
from abc import ABCMeta, abstractmethod
from cffi import FFI
from typing import List

ffi = FFI()


class KinesisDevice(metaclass=ABCMeta):

    @staticmethod
    @abstractmethod
    def list_devices() -> List[str]:
        pass

    def __init__(self, serial_number: str) -> None:
        self._serial_buffer = ffi.new("char[]", serial_number.encode("utf-8"))
        self.list_devices()
        self.open_connection()
        self.start_polling(100)

    def __enter__(self):
        self.open_connection()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close_connection()

    def __del__(self):
        self.close_connection()

    @abstractmethod
    def open_connection(self):
        pass

    @abstractmethod
    def close_connection(self):
        pass

    @abstractmethod
    def check_connection(self) -> bool:
        pass

    @abstractmethod
    def identify(self):
        pass

    @abstractmethod
    def start_polling(self, millies: int) -> bool:
        pass

    @abstractmethod
    def stop_polling(self):
        pass

    @abstractmethod
    def get_polling_duration(self) -> int:
        pass

    @abstractmethod
    def get_hardware_info(self) -> HardwareInfo:
        pass

    @abstractmethod
    def get_software_version(self) -> str:
        pass

    @abstractmethod
    def clear_msg_queue(self):
        pass

    @abstractmethod
    def get_next_msg(self):
        pass

    @abstractmethod
    def wait_for_msg(self):
        pass

    @abstractmethod
    def get_msg_queue_size(self) -> int:
        pass

    def _to_version(self, v):
        major = (v >> 24) & 0xFF
        minor = (v >> 16) & 0xFF
        patch = (v >> 8) & 0xFF
        meta = v & 0xFF
        return major, minor, patch, meta

    software_version = property(get_software_version)
    is_connected = property(check_connection)
    polling_duration = property(get_polling_duration)
    hardware_info = property(get_hardware_info)
    msg_queue_size = property(get_msg_queue_size)