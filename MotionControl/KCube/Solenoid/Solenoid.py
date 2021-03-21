from _thorlabs.motions_control.kcube.solenoid import ffi, lib
from MotionControl.KinesisTypes import *
from MotionControl.KinesisExceptions import check_error_code
from MotionControl.KinesisDevice import KinesisDevice
from typing import List, Callable, Tuple


class KCubeSolenoid(KinesisDevice):

    class StatusBits(NamedTuple):
        solenoid_output_state: bool
        interlock_state: bool
        is_channel_enabled: bool

    @staticmethod
    def list_devices() -> List[str]:
        lib.TLI_BuildDeviceList()
        buffer = ffi.new("char[]", 100)
        lib.TLI_GetDeviceListExt(buffer, len(buffer))

        return ffi.string(buffer).decode("utf-8").split(",")[:-1]

    def open_connection(self):
        check_error_code(lib.SC_Open, self._serial_buffer)

    def close_connection(self):
        check_error_code(lib.SC_Close, self._serial_buffer)

    def check_connection(self) -> bool:
        return lib.SC_CheckConnection(self._serial_buffer)

    def identify(self):
        lib.SC_Identify(self._serial_buffer)

    def start_polling(self, millies: int) -> bool:
        return lib.SC_StartPolling(self._serial_buffer, millies)

    def stop_polling(self):
        check_error_code(lib.SC_StopPolling, self._serial_buffer)

    def get_polling_duration(self) -> int:
        return lib.SC_PollingDuration(self._serial_buffer)

    def request_led_switches(self):
        check_error_code(lib.SC_RequestLEDswitches, self._serial_buffer)

    def get_led_switches(self) -> int:
        return lib.SC_GetLEDswitches(self._serial_buffer)

    def set_led_switches(self, led_switches: int):
        check_error_code(
            lib.SC_SetLEDswitches, self._serial_buffer, led_switches
        )

    def get_software_version(self) -> str:
        v = lib.SC_GetSoftwareVersion(self._serial_buffer)
        major, minor, patch, meta = self._to_version(v)
        return f"{major}.{minor}.{patch}-{meta}"

    def get_hardware_info(self) -> HardwareInfo:
        info = ffi.new("TLI_HardwareInformation *")
        check_error_code(lib.SC_GetHardwareInfoBlock, self._serial_buffer, info)
        return HardwareInfo(
            info.serialNumber,
            ffi.string(info.modelNumber).decode("utf-8"),
            info.type,
            self._to_version(info.firmwareVersion),
            ffi.string(info.notes).decode("utf-8"),
            ffi.buffer(info.deviceDependantData)[:],
            info.hardwareVersion,
            info.modificationState,
            info.numChannels,
        )

    def request_hub_bay(self):
        check_error_code(lib.SC_RequestHubBay, self._serial_buffer)

    def get_hub_bay(self) -> int:
        return lib.SC_GetHubBay(self._serial_buffer)

    def load_settings(self) -> bool:
        return lib.SC_LoadSettings(self._serial_buffer)

    def load_named_settings(self, settings_name: str) -> bool:
        settings_name = ffi.new("char[]", settings_name.encode("utf-8"))
        return lib.SC_LoadNamedSettings(self._serial_buffer, settings_name)

    def persist_settings(self) -> bool:
        return lib.SC_PersistSettings(self._serial_buffer)

    def request_operating_mode(self):
        check_error_code(lib.SC_RequestOperatingMode, self._serial_buffer)

    def get_operating_mode(self) -> OperatingModes:
        return OperatingModes(lib.SC_GetOperatingMode(self._serial_buffer))

    def set_operating_mode(self, mode: OperatingModes):
        mode = ffi.cast("SC_OperatingModes", mode)
        check_error_code(lib.SC_SetOperatingMode, self._serial_buffer, mode)

    def get_solenoid_state(self) -> SolenoidStates:
        return SolenoidStates(lib.SC_GetSolenoidState, self._serial_buffer)

    def request_operating_state(self):
        check_error_code(lib.SC_RequestOperatingState, self._serial_buffer)

    def get_operating_state(self) -> OperatingStates:
        return OperatingStates(lib.SC_GetOperatingState, self._serial_buffer)

    def set_operating_state(self, state: OperatingStates):
        state = ffi.cast("SC_OperatingStates", state)
        check_error_code(lib.SC_SetOperatingState, self._serial_buffer, state)

    def request_cycle_params(self):
        check_error_code(lib.SC_RequestCycleParams, self._serial_buffer)

    def get_cycle_params(self) -> CycleParameters:
        params = ffi.new("SC_CycleParameters  *")
        check_error_code(
            lib.SC_GetCycleParamsBlock, self._serial_buffer, params
        )
        return CycleParameters(
            open_time=params.openTime,
            closed_time=params.closedTime,
            num_cycles=params.numCycles
        )

    def set_cycle_params(self, params: CycleParameters):
        p = ffi.new("SC_CycleParameters  *")
        p.openTime = params.open_time
        p.closedTime = params.closed_time
        p.numCycles = params.num_cycles
        check_error_code(lib.SC_SetCycleParamsBlock, self._serial_buffer, p)

    def request_mmi_params(self):
        check_error_code(lib.SC_RequestMMIParams, self._serial_buffer)

    def get_mmi_params(self) -> Tuple[int, int, int]:
        intensity = ffi.new("__int16 *")
        timeout = ffi.new("__int16 *")
        dim_intensity = ffi.new("__int16 *")
        check_error_code(
            lib.SC_GetMMIParamsExt, self._serial_buffer, intensity, timeout,
            dim_intensity
        )
        return intensity[0], timeout[0], dim_intensity[0]

    def set_mmi_params(self, intensity: int, timout: int, dim_intensity: int):
        check_error_code(
            lib.SC_SetMMIParamsExt, self._serial_buffer, intensity, timout,
            dim_intensity
        )

    def request_trigger_config_params(self):
        check_error_code(lib.SC_RequestTriggerConfigParams, self._serial_buffer)

    def get_trigger_config_params(self) -> TriggerConfigParameters:
        params = ffi.new("KSC_TriggerConfig *")
        check_error_code(
            lib.SC_GetTriggerConfigParamsBlock, self._serial_buffer, params
        )
        return TriggerConfigParameters(
            TriggerPortMode(params.Trigger1Mode),
            TriggerPortPolarity(params.Trigger1Polarity),
            TriggerPortMode(params.Trigger2Mode),
            TriggerPortPolarity(params.Trigger2Polarity)
        )

    def set_trigger_config_params(self, params: TriggerConfigParameters):
        trigger_1_mode = ffi.cast("KSC_TriggerPortMode", params.trigger_1_mode)
        trigger_2_mode = ffi.cast("KSC_TriggerPortMode", params.trigger_2_mode)
        trigger_1_polarity = ffi.cast(
            "KSC_TriggerPortPolarity", params.trigger_1_polarity
        )
        trigger_2_polarity = ffi.cast(
            "KSC_TriggerPortPolarity", params.trigger_2_polarity
        )
        p = ffi.new("KSC_TriggerConfig *")
        p.Trigger1Mode = trigger_1_mode
        p.Trigger2Mode = trigger_2_mode
        p.Trigger1Polarity = trigger_1_polarity
        p.Trigger2Polarity = trigger_2_polarity

        check_error_code(
            lib.SC_SetTriggerConfigParamsBlock, self._serial_buffer, p
        )

    def request_digital_outputs(self):
        check_error_code(lib.SC_RequestDigitalOutputs, self._serial_buffer)

    def get_digital_outputs(self) -> int:
        return lib.SC_GetDigitalOutputs(self._serial_buffer)

    def set_digital_outputs(self, bits: int):
        check_error_code(lib.SC_SetDigitalOutputs, self._serial_buffer, bits)

    def request_status(self):
        check_error_code(lib.SC_RequestStatus, self._serial_buffer)

    def request_status_bits(self):
        check_error_code(lib.SC_RequestStatusBits, self._serial_buffer)

    def get_status_bits(self) -> "KCubeSolenoid.StatusBits":
        status_bits = lib.SC_GetStatusBits(self._serial_buffer)
        bits = self._read_status_bits(status_bits)
        return KCubeSolenoid.StatusBits(
            solenoid_output_state=bits[0],
            interlock_state=bits[13],
            is_channel_enabled=bits[31]
        )

    def request_settings(self):
        check_error_code(lib.SC_RequestSettings, self._serial_buffer)

    def clear_msg_queue(self):
        lib.SC_ClearMessageQueue(self._serial_buffer)

    def get_msg_queue_size(self) -> int:
        return lib.SC_MessageQueueSize(self._serial_buffer)

    def get_next_msg(self):
        msg_type = ffi.new("WORD *")
        msg_id = ffi.new("WORD *")
        msg_data = ffi.new("DWORD *")
        success = lib.SC_GetNextMessage(
            self._serial_buffer, msg_type, msg_id, msg_data
        )
        return success, msg_type[0], msg_id[0], msg_data[0]

    def wait_for_msg(self):
        msg_type = ffi.new("WORD *")
        msg_id = ffi.new("WORD *")
        msg_data = ffi.new("DWORD *")
        success = lib.SC_WaitForMessage(
            self._serial_buffer, msg_type, msg_id, msg_data
        )
        return success, msg_type[0], msg_id[0], msg_data[0]

    def register_message_callback(self, fct: Callable[[], None]):
        global _message_callback
        _message_callback = ffi.def_extern(fct, name="message_callback")
        lib.SC_RegisterMessageCallback(
            self._serial_buffer, lib.message_callback
        )

    led_switches = property(get_led_switches, set_led_switches)
    software_version = property(get_software_version)
    is_connected = property(check_connection)
    polling_duration = property(get_polling_duration)
    hardware_info = property(get_hardware_info)
    msg_queue_size = property(get_msg_queue_size)
    hub_bay = property(get_hub_bay)
    operating_mode = property(get_operating_mode, set_operating_mode)
    solenoid_state = property(get_solenoid_state)
    operating_state = property(get_operating_state, set_operating_state)
    cycle_params = property(get_cycle_params, set_cycle_params)
    trigger_config_params = property(
        get_trigger_config_params, set_trigger_config_params
    )
    digital_outputs = property(get_digital_outputs, set_digital_outputs)
    status_bits = property(get_status_bits)
