from _thorlabs.motions_control.filter_flipper import ffi, lib
from MotionControl.KinesisTypes import *
from MotionControl.KinesisExceptions import check_error_code
from MotionControl.KinesisDevice import KinesisDevice
from typing import List, Callable


class FilterFlipper(KinesisDevice):

    class StatusBits(NamedTuple):
        at_cw_hardware_limit_switch: bool
        at_ccw_hardware_limit_switch: bool
        is_motor_jogging_cw: bool
        is_motor_jogging_ccw: bool
        dig_input_1_state: bool
        dig_input_2_state: bool
        dig_input_3_state: bool
        dig_input_4_state: bool
        dig_input_5_state: bool
        dig_input_6_state: bool
        is_active: bool
        is_channel_enabled: bool

    @staticmethod
    def list_devices() -> List[str]:
        lib.TLI_BuildDeviceList()
        buffer = ffi.new("char[]", 100)
        lib.TLI_GetDeviceListExt(buffer, len(buffer))

        return ffi.string(buffer).decode("utf-8").split(",")[:-1]

    def open_connection(self):
        check_error_code(lib.FF_Open, self._serial_buffer)

    def close_connection(self):
        check_error_code(lib.FF_Close, self._serial_buffer)

    def check_connection(self) -> bool:
        return lib.FF_CheckConnection(self._serial_buffer)

    def identify(self):
        lib.FF_Identify(self._serial_buffer)

    def start_polling(self, millies: int) -> bool:
        return lib.FF_StartPolling(self._serial_buffer, millies)

    def stop_polling(self):
        check_error_code(lib.FF_StopPolling, self._serial_buffer)

    def get_polling_duration(self) -> int:
        return lib.FF_PollingDuration(self._serial_buffer)

    def get_firmware_version(self) -> str:
        v = lib.FF_GetFirmwareVersion(self._serial_buffer)
        major, minor, patch, meta = self._to_version(v)
        return f"{major}.{minor}.{patch}-{meta}"

    def get_software_version(self) -> str:
        v = lib.FF_GetSoftwareVersion(self._serial_buffer)
        major, minor, patch, meta = self._to_version(v)
        return f"{major}.{minor}.{patch}-{meta}"

    def get_hardware_info(self) -> HardwareInfo:
        size_of_model_no = 8
        model_no = ffi.new("char[]", size_of_model_no)
        hardware_type = ffi.new("WORD *")
        num_channels = ffi.new("WORD *")
        size_of_notes = 48
        notes = ffi.new("char[]", size_of_notes)
        firmware_version = ffi.new("DWORD *")
        hardware_version = ffi.new("WORD *")
        modification_state = ffi.new("WORD *")

        check_error_code(
            lib.FF_GetHardwareInfo, self._serial_buffer, model_no,
            size_of_model_no, hardware_type, num_channels, notes, size_of_notes,
            firmware_version, hardware_version, modification_state
        )
        return HardwareInfo(
            ffi.string(self._serial_buffer).decode("utf-8"),
            ffi.string(model_no).decode("utf-8"),
            hardware_type[0],
            self._to_version(firmware_version[0]),
            ffi.string(notes).decode("utf-8"),
            0,
            hardware_version[0],
            modification_state[0],
            num_channels[0],
        )

    def load_settings(self) -> bool:
        return lib.FF_LoadSettings(self._serial_buffer)

    def load_named_settings(self, settings_name: str) -> bool:
        settings_name = ffi.new("char[]", settings_name.encode("utf-8"))
        return lib.FF_LoadNamedSettings(self._serial_buffer, settings_name)

    def persist_settings(self) -> bool:
        return lib.FF_PersistSettings(self._serial_buffer)

    def get_number_positions(self) -> int:
        return lib.FF_GetNumberPositions(self._serial_buffer)

    def home(self):
        check_error_code(lib.FF_Home, self._serial_buffer)

    def move_to_position(self, position: Positions):
        position = ffi.cast("FF_Positions", position)
        check_error_code(lib.FF_MoveToPosition, self._serial_buffer, position)

    def get_position(self) -> Positions:
        return Positions(lib.FF_GetPosition(self._serial_buffer))

    def request_io_settings(self):
        check_error_code(lib.FF_RequestIOSettings, self._serial_buffer)

    def get_io_settings(self) -> IOSettings:
        settings = ffi.new("FF_IOSettings *")
        check_error_code(lib.FF_GetIOSettings, self._serial_buffer, settings)
        return IOSettings(
            settings.transitTime,
            settings.ADCspeedValue,
            IOModes(settings.digIO1OperMode),
            SignalModes(settings.digIO1SignalMode),
            settings.digIO1PulseWidth,
            IOModes(settings.digIO2OperMode),
            SignalModes(settings.digIO2SignalMode),
            settings.digIO2PulseWidth,
        )

    def set_io_settings(self, settings: IOSettings):
        s = ffi.new("FF_IOSettings *")
        dig_io1_op_mode = ffi.cast(
            "FF_IOModes", settings.digital_io1_operation_mode
        )
        dig_io1_sig_mode = ffi.cast(
            "FF_SignalModes", settings.digital_io1_signal_mode
        )
        dig_io2_op_mode = ffi.cast(
            "FF_IOModes", settings.digital_io2_operation_mode
        )
        dig_io2_sig_mode = ffi.cast(
            "FF_SignalModes", settings.digital_io2_signal_mode
        )

        s.transitTime = settings.transit_time
        s.ADCspeedValue = settings.adc_speed_value
        s.digIO1OperMode = dig_io1_op_mode
        s.digIO1SignalMode = dig_io1_sig_mode
        s.digIO1PulseWidth = settings.digital_io1_pulse_width
        s.digIO2OperMode = dig_io2_op_mode
        s.digIO2SignalMode = dig_io2_sig_mode
        s.digIO2PulseWidth = settings.digital_io2_pulse_width

        check_error_code(lib.FF_SetIOSettings, self._serial_buffer, s)

    def get_transit_time(self) -> int:
        return lib.FF_GetTransitTime(self._serial_buffer)

    def set_transit_time(self, millies: int):
        check_error_code(lib.FF_SetTransitTime, millies)

    def request_status(self):
        check_error_code(lib.FF_RequestStatus, self._serial_buffer)

    def get_status_bits(self) -> "FilterFlipper.StatusBits":
        status_bits = lib.FF_GetStatusBits(self._serial_buffer)
        bits = self._read_status_bits(status_bits)
        return FilterFlipper.StatusBits(
            at_cw_hardware_limit_switch=bits[0],
            at_ccw_hardware_limit_switch=bits[1],
            is_motor_jogging_cw=bits[6],
            is_motor_jogging_ccw=bits[7],
            dig_input_1_state=bits[20],
            dig_input_2_state=bits[21],
            dig_input_3_state=bits[22],
            dig_input_4_state=bits[23],
            dig_input_5_state=bits[24],
            dig_input_6_state=bits[25],
            is_active=bits[29],
            is_channel_enabled=bits[31]
        )

    def request_settings(self):
        check_error_code(lib.FF_RequestSettings, self._serial_buffer)

    def clear_msg_queue(self):
        lib.FF_ClearMessageQueue(self._serial_buffer)

    def register_message_callback(self, fct: Callable[[], None]):
        global _message_callback
        _message_callback = ffi.def_extern(fct, name="message_callback")
        lib.FF_RegisterMessageCallback(
            self._serial_buffer, lib.message_callback
        )

    def get_msg_queue_size(self) -> int:
        return lib.FF_MessageQueueSize(self._serial_buffer)

    def get_next_msg(self):
        msg_type = ffi.new("WORD *")
        msg_id = ffi.new("WORD *")
        msg_data = ffi.new("DWORD *")
        success = lib.FF_GetNextMessage(
            self._serial_buffer, msg_type, msg_id, msg_data
        )
        return success, msg_type[0], msg_id[0], msg_data[0]

    def wait_for_msg(self):
        msg_type = ffi.new("WORD *")
        msg_id = ffi.new("WORD *")
        msg_data = ffi.new("DWORD *")
        success = lib.FF_WaitForMessage(
            self._serial_buffer, msg_type, msg_id, msg_data
        )
        return success, msg_type[0], msg_id[0], msg_data[0]

    firmware_version = property(get_firmware_version)
    software_version = property(get_software_version)
    is_connected = property(check_connection)
    polling_duration = property(get_polling_duration)
    hardware_info = property(get_hardware_info)
    msg_queue_size = property(get_msg_queue_size)
    number_positions = property(get_number_positions)
    position = property(get_position)
    io_settings = property(get_io_settings, set_io_settings)
    status_bits = property(get_status_bits)
