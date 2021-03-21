from _thorlabs.motions_control.integrated_stepper_motors import ffi, lib
from MotionControl.KinesisTypes import *
from MotionControl.KinesisExceptions import check_error_code
from MotionControl.KinesisDevice import KinesisDevice
from typing import List


class IntegratedStepperMotors(KinesisDevice):

    class StatusBits(NamedTuple):
        at_cw_hardware_limit_switch: bool
        at_ccw_hardware_limit_switch: bool
        at_cw_software_limit_switch: bool
        at_ccw_software_limit_switch: bool
        is_motor_moving_cw: bool
        is_motor_moving_ccw: bool
        is_motor_jogging_cw: bool
        is_motor_jogging_ccw: bool
        is_motor_connected: bool
        is_motor_homing: bool
        is_motor_homed: bool
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

    def open_connection(self):
        check_error_code(lib.ISC_Open, self._serial_buffer)

    def close_connection(self):
        check_error_code(lib.ISC_Open, self._serial_buffer)

    def identify(self):
        lib.ISC_Identify(self._serial_buffer)

    def enable_channel(self):
        check_error_code(lib.ISC_EnableChannel, self._serial_buffer)

    def disable_channel(self):
        check_error_code(lib.ISC_DisableChannel, self._serial_buffer)

    def home(self):
        check_error_code(lib.ISC_Home, self._serial_buffer)

    def move_relative(self, displacement):
        check_error_code(
            lib.ISC_MoveRelative, self._serial_buffer, displacement
        )

    def move_to_position(self, index: int):
        check_error_code(lib.ISC_MoveToPosition, self._serial_buffer, index)

    def move_absolute(self):
        check_error_code(lib.ISC_MoveAbsolute, self._serial_buffer)

    def move_relative_distance(self):
        check_error_code(lib.ISC_MoveRelativeDistance, self._serial_buffer)

    def move_jog(self, jog_direction: TravelDirection):
        jog_direction = ffi.cast("MOT_TravelDirection", jog_direction)
        check_error_code(lib.ISC_MoveJog, self._serial_buffer, jog_direction)

    def move_at_velocity(self, direction: TravelDirection):
        direction = ffi.cast("MOT_TravelDirection", direction)
        check_error_code(lib.ISC_MoveAtVelocity, self._serial_buffer, direction)

    def stop_immediate(self):
        check_error_code(lib.ISC_StopImmediate, self._serial_buffer)

    def stop_profiled(self):
        check_error_code(lib.ISC_StopProfiled, self._serial_buffer)

    def start_polling(self, millies: int) -> bool:
        return lib.ISC_StartPolling(self._serial_buffer, millies)

    def stop_polling(self):
        check_error_code(lib.ISC_StopPolling, self._serial_buffer)

    def load_settings(self) -> bool:
        return lib.ISC_LoadSettings(self._serial_buffer)

    def load_named_settings(self, settings_name: str) -> bool:
        settings_name = ffi.new("char[]", settings_name.encode("utf-8"))
        return lib.ISC_LoadNamedSettings(self._serial_buffer, settings_name)

    def persist_settings(self) -> bool:
        return lib.ISC_PersistSettings(self._serial_buffer)

    def reset_stage_to_default(self):
        check_error_code(lib.ISC_ResetStageToDefaults, self._serial_buffer)

    def real_value_to_device_unit(
        self, real_value: float, unit: UnitType
    ) -> int:
        dev_unit = ffi.new("int *")
        check_error_code(
            lib.ISC_GetDeviceUnitFromRealValue, self._serial_buffer, real_value,
            dev_unit, unit
        )
        return dev_unit[0]

    def device_unit_to_real_value(
        self, device_unit: int, unit: UnitType
    ) -> float:
        real_value = ffi.new("double *")
        check_error_code(
            lib.ISC_GetRealValueFromDeviceUnit, self._serial_buffer,
            device_unit, real_value, unit
        )
        return real_value[0]

    def get_firmware_version(self) -> str:
        v = lib.ISC_GetFirmwareVersion(self._serial_buffer)
        major, minor, patch, meta = self._to_version(v)
        return f"{major}.{minor}.{patch}-{meta}"

    def get_software_version(self) -> str:
        v = lib.ISC_GetSoftwareVersion(self._serial_buffer)
        major, minor, patch, meta = self._to_version(v)
        return f"{major}.{minor}.{patch}-{meta}"

    def check_connection(self) -> bool:
        return lib.ISC_CheckConnection(self._serial_buffer)

    def get_can_home(self) -> bool:
        return lib.ISC_CanHome(self._serial_buffer)

    def get_can_move_without_homing_first(self) -> bool:
        return lib.ISC_CanMoveWithoutHomingFirst(self._serial_buffer)

    def get_polling_duration(self) -> int:
        return lib.ISC_PollingDuration(self._serial_buffer)

    def get_hardware_info(self) -> HardwareInfo:
        info = ffi.new("TLI_HardwareInformation *")
        check_error_code(
            lib.ISC_GetHardwareInfoBlock, self._serial_buffer, info
        )
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

    def get_position(self) -> int:
        return lib.ISC_GetPosition(self._serial_buffer)

    def get_is_calibration_active(self) -> bool:
        return lib.ISC_IsCalibrationActive(self._serial_buffer)

    def get_number_of_positions(self) -> int:
        return lib.ISC_GetNumberPositions(self._serial_buffer)

    def get_status_bits(self) -> "IntegratedStepperMotors.StatusBits":
        status_bits = lib.ISC_GetStatusBits(self._serial_buffer)
        bits = self._read_status_bits(status_bits)
        return IntegratedStepperMotors.StatusBits(
            at_cw_hardware_limit_switch=bits[0],
            at_ccw_hardware_limit_switch=bits[1],
            at_cw_software_limit_switch=bits[2],
            at_ccw_software_limit_switch=bits[3],
            is_motor_moving_cw=bits[4],
            is_motor_moving_ccw=bits[5],
            is_motor_jogging_cw=bits[6],
            is_motor_jogging_ccw=bits[7],
            is_motor_connected=bits[8],
            is_motor_homing=bits[9],
            is_motor_homed=bits[10],
            dig_input_1_state=bits[20],
            dig_input_2_state=bits[21],
            dig_input_3_state=bits[22],
            dig_input_4_state=bits[23],
            dig_input_5_state=bits[24],
            dig_input_6_state=bits[25],
            is_active=bits[29],
            is_channel_enabled=bits[31]
        )

    def reset_rotation_modes(self):
        check_error_code(lib.ISC_ResetRotationModes, self._serial_buffer)

    def request_homing_params(self):
        check_error_code(lib.ISC_RequestHomingParams, self._serial_buffer)

    def request_jog_params(self):
        check_error_code(lib.ISC_RequestJogParams, self._serial_buffer)

    def request_limit_switch_params(self):
        check_error_code(lib.ISC_RequestLimitSwitchParams, self._serial_buffer)

    def request_potentiometer_params(self):
        check_error_code(
            lib.ISC_RequestPotentiometerParams, self._serial_buffer
        )

    def request_button_params(self):
        check_error_code(lib.ISC_RequestButtonParams, self._serial_buffer)

    def request_power_params(self):
        check_error_code(lib.ISC_RequestPowerParams, self._serial_buffer)

    def request_velocity_params(self):
        check_error_code(lib.ISC_RequestVelParams, self._serial_buffer)

    def request_backlash(self):
        check_error_code(lib.ISC_RequestBacklash, self._serial_buffer)

    def request_move_absolute_position(self):
        check_error_code(
            lib.ISC_RequestMoveAbsolutePosition, self._serial_buffer
        )

    def request_move_relative_distance(self):
        check_error_code(
            lib.ISC_RequestMoveRelativeDistance, self._serial_buffer
        )

    def request_bow_index(self):
        check_error_code(lib.ISC_RequestBowIndex, self._serial_buffer)

    def request_trigger_switches(self):
        check_error_code(lib.ISC_RequestTriggerSwitches, self._serial_buffer)

    def request_position(self):
        check_error_code(lib.ISC_RequestPosition, self._serial_buffer)

    def request_status_bits(self):
        check_error_code(lib.ISC_RequestStatusBits, self._serial_buffer)

    def request_status(self):
        check_error_code(lib.ISC_RequestStatus, self._serial_buffer)

    def request_settings(self):
        check_error_code(lib.ISC_RequestSettings, self._serial_buffer)

    def set_direction(self, reverse: bool):
        lib.ISC_SetDirection(self._serial_buffer, reverse)

    def set_rotation_modes(
        self, mode: MovementModes, direction: MovementDirections
    ):
        mode = ffi.cast("MOT_MovementModes", mode)
        direction = ffi.cast("MOT_MovementDirections", direction)
        check_error_code(
            lib.ISC_SetRotationModes, self._serial_buffer, mode, direction
        )

    def set_calibration_file(self, filename: str, enabled: bool = True):
        filename = ffi.new("char[]", filename.encode("utf-8"))
        lib.ISC_SetCalibrationFile(self._serial_buffer, filename, enabled)

    def get_calibration_file(self) -> str:
        size = 100
        filename = ffi.new("char[]", size)
        while not lib.ISC_GetCalibrationFile(
            self._serial_buffer, filename, size
        ):
            size += 100
            filename = ffi.new("char[]", size)

        return ffi.string(filename).decode("utf-8")

    def get_potentiometer_params(self, index: int) -> PotentiometerParameters:
        threshold_deflection = ffi.new("WORD *")
        velocity = ffi.new("DWORD *")
        check_error_code(
            lib.ISC_GetPotentiometerParams, self._serial_buffer, index,
            threshold_deflection, velocity
        )
        return PotentiometerParameters(
            index, threshold_deflection[0], velocity[0]
        )

    def set_potentiometer_params(self, params: PotentiometerParameters):
        threshold_deflection = ffi.cast("WORD", params.threshold_deflection)
        velocity = ffi.cast("DWORD", params.velocity)
        check_error_code(
            lib.ISC_SetPotentiometerParams, self._serial_buffer, params.index,
            threshold_deflection, velocity
        )

    def get_homing_velocity(self) -> int:
        return lib.ISC_GetHomingVelocity(self._serial_buffer)

    def set_homing_velocity(self, velocity: int):
        check_error_code(
            lib.ISC_SetHomingVelocity, self._serial_buffer, velocity
        )

    def get_led_switches(self) -> int:
        return lib.ISC_GetLEDswitches(self._serial_buffer)

    def set_led_switches(self, led_switches: int):
        check_error_code(
            lib.ISC_SetLEDswitches, self._serial_buffer, led_switches
        )

    def get_jog_step_size(self) -> int:
        return lib.ISC_GetJogStepSize(self._serial_buffer)

    def set_jog_step_size(self, step_size: int):
        check_error_code(lib.ISC_SetJogStepSize, self._serial_buffer, step_size)

    def get_backlash(self) -> int:
        return lib.ISC_GetBacklash(self._serial_buffer)

    def set_backlash(self, distance: int):
        check_error_code(lib.ISC_SetBacklash, self._serial_buffer, distance)

    def get_position_counter(self) -> int:
        return lib.ISC_GetPositionCounter(self._serial_buffer)

    def set_position_counter(self, count: int):
        check_error_code(lib.ISC_SetPositionCounter, self._serial_buffer, count)

    def get_limit_switch_params(self) -> LimitSwitchParameters:
        clockwise_hardware_limit = ffi.new("MOT_LimitSwitchModes * ")
        anticlockwise_hardware_limit = ffi.new("MOT_LimitSwitchModes * ")
        clockwise_position = ffi.new("unsigned int * ")
        anticlockwise_position = ffi.new("unsigned int * ")
        soft_limit_mode = ffi.new("MOT_LimitSwitchSWModes * ")

        check_error_code(
            lib.ISC_GetLimitSwitchParams, self._serial_buffer,
            clockwise_hardware_limit, anticlockwise_hardware_limit,
            clockwise_position, anticlockwise_position, soft_limit_mode
        )

        return LimitSwitchParameters(
            LimitSwitchModes(clockwise_hardware_limit[0]),
            LimitSwitchModes(anticlockwise_hardware_limit[0]),
            clockwise_position[0], anticlockwise_position[0],
            LimitSwitchSWModes(soft_limit_mode[0])
        )

    def set_limit_switch_params(self, params: LimitSwitchParameters):
        clockwise_hardware_limit = ffi.cast(
            "MOT_LimitSwitchModes", params.clockwise_hardware_limit
        )
        anticlockwise_hardware_limit = ffi.cast(
            "MOT_LimitSwitchModes", params.anticlockwise_hardware_limit
        )
        clockwise_position = ffi.cast("unsigned int", params.clockwise_position)
        anticlockwise_position = ffi.cast(
            "unsigned int", params.anticlockwise_position
        )
        soft_limit_mode = ffi.cast(
            "MOT_LimitSwitchSWModes", params.soft_limit_mode
        )
        check_error_code(
            lib.ISC_SetLimitSwitchParams, self._serial_buffer,
            clockwise_hardware_limit, anticlockwise_hardware_limit,
            clockwise_position, anticlockwise_position, soft_limit_mode
        )

    def get_soft_limit_mode(self) -> LimitsSoftwareApproachPolicy:
        return LimitsSoftwareApproachPolicy(
            lib.ISC_GetSoftLimitMode(self._serial_buffer)
        )

    def set_soft_limit_mode(
        self, limitsSoftwareApproachPolicy: LimitsSoftwareApproachPolicy
    ):
        limitsSoftwareApproachPolicy = ffi.cast(
            "MOT_LimitsSoftwareApproachPolicy", limitsSoftwareApproachPolicy
        )
        lib.ISC_SetLimitsSoftwareApproachPolicy(
            self._serial_buffer, limitsSoftwareApproachPolicy
        )

    def get_button_params(self) -> ButtonParameters:
        button_mode = ffi.new("MOT_ButtonModes *")
        left_button_position = ffi.new("int *")
        right_button_position = ffi.new("int *")
        timeout = ffi.new("short *")
        check_error_code(
            lib.ISC_GetButtonParams, self._serial_buffer, button_mode,
            left_button_position, right_button_position, timeout
        )
        return ButtonParameters(
            ButtonModes(button_mode[0]), left_button_position[0],
            right_button_position[0], timeout[0]
        )

    def set_button_params(self, params: ButtonParameters):
        button_mode = ffi.cast("MOT_ButtonModes", params.button_mode)
        check_error_code(
            lib.ISC_SetButtonParams, self._serial_buffer, button_mode,
            params.left_button_position, params.right_button_position
        )

    def get_absolute_position(self) -> int:
        return lib.ISC_GetMoveAbsolutePosition(self._serial_buffer)

    def set_absolute_position(self, position: int):
        check_error_code(
            lib.ISC_SetMoveAbsolutePosition, self._serial_buffer, position
        )

    def get_relative_distance(self) -> int:
        return lib.ISC_GetMoveRelativeDistance(self._serial_buffer)

    def set_relative_distance(self, distance: int):
        check_error_code(
            lib.ISC_SetMoveRelativeDistance, self._serial_buffer, distance
        )

    def get_homing_params(self) -> HomingParameters:
        params = ffi.new("MOT_HomingParameters *")
        check_error_code(
            lib.ISC_GetHomingParamsBlock, self._serial_buffer, params
        )
        return HomingParameters(
            TravelDirection(params.direction),
            HomeLimitSwitchDirection(params.limitSwitch), params.velocity,
            params.offsetDistance
        )

    def set_homing_params(self, params: HomingParameters):
        direction = ffi.cast("MOT_TravelDirection", params.direction)
        limit_switch = ffi.cast(
            "MOT_HomeLimitSwitchDirection", params.limit_switch
        )

        p = ffi.new("MOT_HomingParameters *")
        p.direction = direction
        p.limitSwitch = limit_switch
        p.velocity = params.velocity
        p.offsetDistance = params.offset_distance

        check_error_code(lib.ISC_SetHomingParamsBlock, self._serial_buffer, p)

    def get_jog_params(self) -> JogParameters:
        params = ffi.new("MOT_JogParameters *")
        check_error_code(lib.ISC_GetJogParamsBlock, self._serial_buffer, params)
        return JogParameters(
            JogModes(params.mode), params.stepSize,
            VelocityParameters(
                params.velParams.minVelocity, params.velParams.acceleration,
                params.velParams.maxVelocity
            ), StopModes(params.stopMode)
        )

    def set_jog_params(self, params: JogParameters):
        mode = ffi.cast("MOT_JogModes", params.mode)
        vel_params = ffi.new("MOT_VelocityParameters *")
        vel_params.minVelocity = params.vel_params.min_velocity
        vel_params.acceleration = params.vel_params.acceleration
        vel_params.maxVelocity = params.vel_params.max_velocity
        stop_mode = ffi.cast("MOT_StopModes", params.stop_mode)

        p = ffi.new("MOT_JogParameters *")
        p.mode = mode
        p.stepSize = params.step_size
        p.velParams = vel_params[0]
        p.stopMode = stop_mode

        check_error_code(lib.ISC_SetJogParamsBlock, self._serial_buffer, p)

    def get_velocity_params(self) -> VelocityParameters:
        params = ffi.new("MOT_VelocityParameters *")
        check_error_code(lib.ISC_GetVelParamsBlock, self._serial_buffer, params)
        return VelocityParameters(
            params.acceleration, params.maxVelocity, params.minVelocity
        )

    def set_velocity_params(self, params: VelocityParameters):
        p = ffi.new("MOT_VelocityParameters *")
        p.minVelocity = params.min_velocity
        p.acceleration = params.acceleration
        p.maxVelocity = params.max_velocity
        check_error_code(lib.ISC_SetVelParamsBlock, self._serial_buffer, p)

    def get_power_params(self) -> PowerParameters:
        params = ffi.new("MOT_PowerParameters *")
        check_error_code(lib.ISC_GetPowerParams, self._serial_buffer, params)
        return PowerParameters(params.restPercentage, params.movePercentage)

    def set_power_params(self, params: PowerParameters):
        p = ffi.new("MOT_PowerParameters *")
        p.restPercentage = params.rest_percentage
        p.movePercentage = params.move_percentage
        check_error_code(lib.ISC_SetPowerParams, self._serial_buffer, p)

    def get_bow_index(self) -> int:
        return lib.ISC_GetBowIndex(self._serial_buffer)

    def set_bow_index(self, index: int) -> int:
        check_error_code(lib.ISC_SetBowIndex, self._serial_buffer, index)

    def get_trigger_switches(self) -> int:
        return lib.ISC_GetTriggerSwitches(self._serial_buffer)

    def set_trigger_switches(self, indicator_bits: int):
        check_error_code(
            lib.ISC_SetTriggerSwitches, self._serial_buffer, indicator_bits
        )

    def get_stage_axis_limits(self) -> TravelLimits:
        return TravelLimits(
            lib.ISC_GetStageAxisMinPos(self._serial_buffer),
            lib.ISC_GetStageAxisMaxPos(self._serial_buffer)
        )

    def set_stage_axis_limits(self, limits: TravelLimits):
        check_error_code(
            lib.ISC_SetStageAxisLimits, self._serial_buffer,
            limits.min_position, limits.max_position
        )

    def get_motor_travel_mode(self) -> TravelModes:
        return TravelModes(lib.ISC_GetMotorTravelMode(self._serial_buffer))

    def set_motor_travel_mode(self, mode: TravelModes):
        mode = ffi.cast("MOT_TravelModes", mode)
        check_error_code(lib.ISC_SetMotorTravelMode, self._serial_buffer, mode)

    def get_motor_params(self) -> MotorParameters:
        steps_per_rev = ffi.new("double *")
        gear_box_ratio = ffi.new("double *")
        pitch = ffi.new("double *")
        check_error_code(
            lib.ISC_GetMotorParamsExt, self._serial_buffer, steps_per_rev,
            gear_box_ratio, pitch
        )
        return MotorParameters(steps_per_rev[0], gear_box_ratio[0], pitch[0])

    def set_motor_params(self, params: MotorParameters):
        check_error_code(
            lib.ISC_SetMotorParamsExt, self._serial_buffer,
            params.steps_per_rev, params.gear_box_ratio, params.pitch
        )

    firmware_version = property(get_firmware_version)
    software_version = property(get_software_version)
    status_bits = property(get_status_bits)
    is_connected = property(check_connection)
    can_home = property(get_can_home)
    can_move_without_homing_first = property(get_can_move_without_homing_first)
    polling_duration = property(get_polling_duration)
    hardware_info = property(get_hardware_info)
    is_calibration_active = property(get_is_calibration_active)
    number_of_positions = property(get_number_of_positions)
    calibration_file = property(get_calibration_file, set_calibration_file)
    position = property(get_position)

    homing_velocity = property(get_homing_velocity, set_homing_velocity)
    led_switches = property(get_led_switches, set_led_switches)
    jog_step_size = property(get_jog_step_size, set_jog_step_size)
    backlash = property(get_backlash, set_backlash)
    position_counter = property(get_position_counter, set_position_counter)
    soft_limit_mode = property(get_soft_limit_mode, set_soft_limit_mode)
    absolute_position = property(get_absolute_position, set_absolute_position)
    relative_distance = property(get_relative_distance, set_relative_distance)
    homing_params = property(get_homing_params, set_homing_params)
    jog_params = property(get_jog_params, set_jog_params)
    limit_switch_params = property(
        get_limit_switch_params, set_limit_switch_params
    )
    button_params = property(get_button_params, set_button_params)
    potentiometer_params = property(
        get_potentiometer_params, set_potentiometer_params
    )
    velocity_params = property(get_velocity_params, set_velocity_params)
    power_params = property(get_power_params, set_power_params)
    bow_index = property(get_bow_index, set_bow_index)
    trigger_switches = property(get_trigger_switches, set_trigger_switches)
    stage_axis_limits = property(get_stage_axis_limits, set_stage_axis_limits)
    motor_travel_mode = property(get_motor_travel_mode, set_motor_travel_mode)

    def clear_msg_queue(self):
        lib.ISC_ClearMessageQueue(self._serial_buffer)

    def get_next_msg(self):
        msg_type = ffi.new("WORD *")
        msg_id = ffi.new("WORD *")
        msg_data = ffi.new("DWORD *")
        success = lib.ISC_GetNextMessage(
            self._serial_buffer, msg_type, msg_id, msg_data
        )
        return success, msg_type[0], msg_id[0], msg_data[0]

    def wait_for_msg(self):
        msg_type = ffi.new("WORD *")
        msg_id = ffi.new("WORD *")
        msg_data = ffi.new("DWORD *")
        success = lib.ISC_WaitForMessage(
            self._serial_buffer, msg_type, msg_id, msg_data
        )
        return success, msg_type[0], msg_id[0], msg_data[0]

    def get_msg_queue_size(self) -> int:
        return lib.ISC_MessageQueueSize(self._serial_buffer)

    def _get_has_last_msg_timer_overrun(self) -> bool:
        return lib.ISC_HasLastMsgTimerOverrun(self._serial_buffer)

    msg_queue_size = property(get_msg_queue_size)
    _has_last_msg_timer_overrun = property(_get_has_last_msg_timer_overrun)

    def _to_version(self, v):
        major = (v >> 24) & 0xFF
        minor = (v >> 16) & 0xFF
        patch = (v >> 8) & 0xFF
        meta = v & 0xFF
        return major, minor, patch, meta


if __name__ == "__main__":
    lib.TLI_InitializeSimulations()

    print(IntegratedStepperMotors.list_devices())
    dev = IntegratedStepperMotors("45000001")
    print(f"is connected: {dev.is_connected}")
    print(f"firmware: {dev.firmware_version}")
    print(f"software: {dev.software_version}")
    print(f"can home: {dev.can_home}")
    print(f"can move without homing first: {dev.can_move_without_homing_first}")
    dev.homing_velocity = 1_000_000_000

    dev.clear_msg_queue()
    dev.home()
    print("dev is homing")
    succ, msg_type, msg_id, _ = dev.wait_for_msg()
    while (not succ) or (msg_type != MessageType.GenericMotor
                        ) or (msg_id != GenericMotorsMID.Homed):
        succ, msg_type, msg_id, _ = dev.wait_for_msg()

    print("dev homed")

    dev.clear_msg_queue()
    dev.move_to_position(10)
    print("dev is moving to position")
    succ, msg_type, msg_id, _ = dev.wait_for_msg()
    while (not succ) or (msg_type != MessageType.GenericMotor
                        ) or (msg_id != GenericMotorsMID.Moved):
        succ, msg_type, msg_id, _ = dev.wait_for_msg()
    print(dev.position)
    dev.move_relative(100_000)
    succ, msg_type, msg_id, _ = dev.wait_for_msg()
    while (not succ) or (msg_type != MessageType.GenericMotor
                        ) or (msg_id != GenericMotorsMID.Moved):
        succ, msg_type, msg_id, _ = dev.wait_for_msg()
    print(dev.position)

    print(dev.hardware_info)

    lib.TLI_UninitializeSimulations()