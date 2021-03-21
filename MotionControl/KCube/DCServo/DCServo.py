from _thorlabs.motions_control.kcube.dc_servo import ffi, lib
from MotionControl.KinesisTypes import *
from MotionControl.KinesisExceptions import check_error_code
from MotionControl.KinesisDevice import KinesisDevice
from typing import List, Callable


class KCubeDCServo(KinesisDevice):

    class StatusBits(NamedTuple):
        at_cw_hardware_limit_switch: bool
        at_ccw_hardware_limit_switch: bool
        is_motor_moving_cw: bool
        is_motor_moving_ccw: bool
        is_motor_jogging_cw: bool
        is_motor_jogging_ccw: bool
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

    def open_connection(self):
        check_error_code(lib.CC_Open, self._serial_buffer)

    def close_connection(self):
        check_error_code(lib.CC_Open, self._serial_buffer)

    def check_connection(self) -> bool:
        return lib.CC_CheckConnection(self._serial_buffer)

    def identify(self):
        lib.CC_Identify(self._serial_buffer)

    def start_polling(self, millies: int) -> bool:
        return lib.CC_StartPolling(self._serial_buffer, millies)

    def stop_polling(self):
        check_error_code(lib.CC_StopPolling, self._serial_buffer)

    def get_polling_duration(self) -> int:
        return lib.CC_PollingDuration(self._serial_buffer)

    def request_led_switches(self):
        check_error_code(lib.CC_RequestLEDswitches, self._serial_buffer)

    def get_led_switches(self) -> int:
        return lib.CC_GetLEDswitches(self._serial_buffer)

    def set_led_switches(self, led_switches: int):
        check_error_code(
            lib.CC_SetLEDswitches, self._serial_buffer, led_switches
        )

    def get_software_version(self) -> str:
        v = lib.CC_GetSoftwareVersion(self._serial_buffer)
        major, minor, patch, meta = self._to_version(v)
        return f"{major}.{minor}.{patch}-{meta}"

    def get_hardware_info(self) -> HardwareInfo:
        info = ffi.new("TLI_HardwareInformation *")
        check_error_code(lib.CC_GetHardwareInfoBlock, self._serial_buffer, info)
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

    def get_hub_bay(self) -> int:
        return lib.CC_GetHubBay(self._serial_buffer)

    def load_settings(self) -> bool:
        return lib.CC_LoadSettings(self._serial_buffer)

    def load_named_settings(self, settings_name: str) -> bool:
        settings_name = ffi.new("char[]", settings_name.encode("utf-8"))
        return lib.CC_LoadNamedSettings(self._serial_buffer, settings_name)

    def persist_settings(self) -> bool:
        return lib.CC_PersistSettings(self._serial_buffer)

    def reset_stage_to_defaults(self):
        check_error_code(lib.CC_ResetStageToDefaults, self._serial_buffer)

    def disable_channel(self):
        check_error_code(lib.CC_DisableChannel, self._serial_buffer)

    def enable_channel(self):
        check_error_code(lib.CC_EnableChannel, self._serial_buffer)

    def get_can_device_lock_front_panel(self) -> bool:
        return lib.CC_CanDeviceLockFrontPanel(self._serial_buffer)

    def request_front_panel_locked(self):
        check_error_code(lib.CC_RequestFrontPanelLocked, self._serial_buffer)

    def get_front_panel_locked(self) -> bool:
        return lib.CC_GetFrontPanelLocked(self._serial_buffer)

    def set_front_panel_locked(self, locked: bool):
        check_error_code(lib.CC_SetFrontPanelLock, self._serial_buffer, locked)

    def get_number_positions(self) -> int:
        return lib.CC_GetNumberPositions(self._serial_buffer)

    def move_to_position(self, index: int):
        check_error_code(lib.CC_MoveToPosition, self._serial_buffer, index)

    def request_position(self):
        check_error_code(lib.CC_RequestPosition, self._serial_buffer)

    def get_position(self) -> int:
        return lib.CC_GetPosition(self._serial_buffer)

    def get_can_home(self) -> bool:
        return lib.CC_CanHome(self._serial_buffer)

    def get_can_move_without_homing_first(self):
        return lib.CC_CanMoveWithoutHomingFirst(self._serial_buffer)

    def home(self):
        check_error_code(lib.CC_Home, self._serial_buffer)

    def register_message_callback(self, fct: Callable[[], None]):
        global _message_callback
        _message_callback = ffi.def_extern(fct, name="message_callback")
        lib.CC_RegisterMessageCallback(
            self._serial_buffer, lib.message_callback
        )

    def request_homing_params(self):
        check_error_code(lib.CC_RequestHomingParams, self._serial_buffer)

    def get_homing_params(self) -> HomingParameters:
        params = ffi.new("MOT_HomingParameters *")
        check_error_code(
            lib.CC_GetHomingParamsBlock, self._serial_buffer, params
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

        check_error_code(lib.CC_SetHomingParamsBlock, self._serial_buffer, p)

    def move_relative(self, displacement: int):
        check_error_code(lib.CC_MoveRelative, self._serial_buffer, displacement)

    def request_jog_params(self):
        check_error_code(lib.CC_RequestJogParams, self._serial_buffer)

    def get_jog_params(self) -> JogParameters:
        params = ffi.new("MOT_JogParameters *")
        check_error_code(lib.CC_GetJogParamsBlock, self._serial_buffer, params)

        return JogParameters(
            JogModes(params.mode), params.stepSize,
            VelocityParameters(
                params.velParams.acceleration,
                params.velParams.maxVelocity,
                params.velParams.minVelocity,
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

        check_error_code(lib.CC_SetJogParamsBlock, self._serial_buffer, p)

    def move_jog(self, direction: TravelDirection):
        direction = ffi.cast("MOT_TravelDirection", direction)
        check_error_code(lib.CC_MoveJog, self._serial_buffer, direction)

    def request_velocity_params(self):
        check_error_code(lib.CC_RequestVelParams, self._serial_buffer)

    def get_velocity_params(self) -> VelocityParameters:
        params = ffi.new("MOT_VelocityParameters *")
        check_error_code(lib.CC_GetVelParamsBlock, self._serial_buffer, params)

        return VelocityParameters(
            params.acceleration,
            params.maxVelocity,
            params.minVelocity,
        )

    def set_velocity_params(self, params: VelocityParameters):
        p = ffi.new("MOT_VelocityParameters *")
        p.minVelocity = params.min_velocity
        p.acceleration = params.acceleration
        p.maxVelocity = params.max_velocity

        check_error_code(lib.CC_SetVelParamsBlock, self._serial_buffer, p)

    def move_at_velocity(self, direction: TravelDirection):
        direction = ffi.cast("MOT_TravelDirection", direction)
        check_error_code(lib.CC_MoveAtVelocity, self._serial_buffer, direction)

    def set_direction(self, reverse: bool):
        check_error_code(lib.CC_SetDirection, self._serial_buffer, reverse)

    def stop_immediate(self):
        check_error_code(lib.CC_StopImmediate, self._serial_buffer)

    def stop_profiled(self):
        check_error_code(lib.CC_StopProfiled, self._serial_buffer)

    def request_backlash(self):
        check_error_code(lib.CC_RequestBacklash, self._serial_buffer)

    def get_backlash(self) -> int:
        return lib.CC_GetBacklash(self._serial_buffer)

    def set_backlash(self, distance: int):
        check_error_code(lib.CC_SetBacklash, self._serial_buffer, distance)

    def get_position_counter(self) -> int:
        return lib.CC_GetPositionCounter(self._serial_buffer)

    def set_position_counter(self, count: int):
        check_error_code(lib.CC_SetPositionCounter, self._serial_buffer, count)

    def request_encoder_counter(self):
        check_error_code(lib.CC_RequestEncoderCounter, self._serial_buffer)

    def get_encoder_counter(self) -> int:
        return lib.CC_GetEncoderCounter(self._serial_buffer)

    def set_encoder_counter(self, count: int):
        check_error_code(lib.CC_SetEncoderCounter, self._serial_buffer, count)

    def request_limit_switch_params(self):
        check_error_code(lib.CC_RequestLimitSwitchParams, self._serial_buffer)

    def get_limit_switch_params(self) -> LimitSwitchParameters:
        params = ffi.new("MOT_LimitSwitchParameters *")
        check_error_code(
            lib.CC_GetLimitSwitchParamsBlock, self._serial_buffer, params
        )
        return LimitSwitchParameters(
            LimitSwitchModes(params.clockwiseHardwareLimit),
            LimitSwitchModes(params.anticlockwiseHardwareLimit),
            params.clockwisePosition, params.anticlockwisePosition,
            LimitSwitchSWModes(params.softLimitMode)
        )

    def set_limit_switch_params(self, params: LimitSwitchParameters):
        cw_hl = ffi.cast(
            "MOT_LimitSwitchModes", params.clockwise_hardware_limit
        )
        ccw_hl = ffi.cast(
            "MOT_LimitSwitchModes", params.anticlockwise_hardware_limit
        )
        soft_limit_mode = ffi.cast(
            "MOT_LimitSwitchSWModes", params.soft_limit_mode
        )

        p = ffi.new("MOT_LimitSwitchParameters *")
        p.clockwiseHardwareLimit = cw_hl
        p.anticlockwiseHardwareLimit = ccw_hl
        p.clockwisePosition = params.clockwise_position
        p.anticlockwisePosition = params.anticlockwise_position
        p.softLimitMode = soft_limit_mode

        check_error_code(
            lib.CC_SetLimitSwitchParamsBlock, self._serial_buffer, p
        )

    def get_software_limit_mode(self) -> LimitsSoftwareApproachPolicy:
        return LimitsSoftwareApproachPolicy(
            lib.CC_GetSoftLimitMode(self._serial_buffer)
        )

    def set_softwarre_limit_mode(self, mode: LimitsSoftwareApproachPolicy):
        mode = ffi.cast("MOT_LimitsSoftwareApproachPolicy", mode)
        lib.CC_SetLimitsSoftwareApproachPolicy(self._serial_buffer, mode)

    def request_mmi_params(self):
        check_error_code(lib.CC_RequestMMIParams, self._serial_buffer)

    def get_mmi_params(self) -> MMIParameters:
        params = ffi.new("KMOT_MMIParams *")
        check_error_code(lib.CC_GetMMIParamsBlock, self._serial_buffer, params)
        return MMIParameters(
            WheelMode(params.WheelMode),
            params.WheelMaxVelocity, params.WheelAcceleration,
            DirectionSense(params.WheelDirectionSense), params.PresetPos1,
            params.PresetPos2, params.DisplayIntensity, params.DisplayTimeout,
            params.DisplayDimIntensity
        )

    def set_mmi_params(self, params: MMIParameters):
        wheel_mode = ffi.cast("KMOT_WheelMode", params.wheel_mode)
        wheel_direction_sense = ffi.cast(
            "KMOT_WheelMode", params.wheel_direction_sense
        )
        p = ffi.new("KMOT_MMIParams *")
        p.WheelMode = wheel_mode
        p.WheelMaxVelocity = params.wheel_max_velocity
        p.WheelAcceleration = params.wheel_acceleration
        p.WheelDirectionSense = wheel_direction_sense
        p.PresetPos1 = params.preset_pos_1
        p.PresetPos2 = params.preset_pos_2
        p.DisplayIntensity = params.display_intensity
        p.DisplayTimeout = params.display_timeout
        p.DisplayDimIntensity = params.display_dim_intensity

        check_error_code(lib.CC_SetMMIParamsBlock, self._serial_buffer, p)

    def request_trigger_config_params(self):
        check_error_code(lib.CC_RequestTriggerConfigParams, self._serial_buffer)

    def get_trigger_config_params(self) -> TriggerConfigParameters:
        params = ffi.new("KSC_TriggerConfig *")
        check_error_code(
            lib.CC_GetTriggerConfigParamsBlock, self._serial_buffer, params
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
            lib.CC_SetTriggerConfigParamsBlock, self._serial_buffer, p
        )

    def request_position_trigger_params(self):
        check_error_code(lib.CC_RequestPosTriggerParams, self._serial_buffer)

    def get_position_trigger_params(self) -> PositionTriggerParameters:
        params = ffi.new("KMOT_TriggerParams *")
        check_error_code(
            lib.CC_GetTriggerParamsParamsBlock, self._serial_buffer, params
        )
        return PositionTriggerParameters(
            params.TriggerStartPositionFwd, params.TriggerIntervalFwd,
            params.TriggerPulseCountFwd, params.TriggerStartPositionRev,
            params.TriggerIntervalRev, params.TriggerPulseCountRev,
            params.TriggerPulseWidth, params.CycleCount
        )

    def set_position_trigger_params(self, params: PositionTriggerParameters):
        p = ffi.new("KMOT_TriggerParams *")
        p.TriggerStartPositionFwd = params.trigger_start_position_fwd
        p.TriggerIntervalFwd = params.trigger_interval_fwd
        p.TriggerPulseCountFwd = params.trigger_pulse_count_fwd
        p.TriggerStartPositionRev = params.trigger_start_position_fwd
        p.TriggerIntervalRev = params.trigger_interval_rev
        p.TriggerPulseCountRev = params.trigger_pulse_count_rev
        p.TriggerPulseWidth = params.trigger_pulse_width
        p.CycleCount = params.cycle_count

        check_error_code(
            lib.CC_SetTriggerParamsParamsBlock, self._serial_buffer, p
        )

    def request_move_absolute_position(self):
        check_error_code(
            lib.CC_RequestMoveAbsolutePosition, self._serial_buffer
        )

    def get_move_absolute_position(self) -> int:
        return lib.CC_GetMoveAbsolutePosition(self._serial_buffer)

    def set_move_absolute_position(self, position: int):
        check_error_code(
            lib.CC_SetMoveAbsolutePosition, self._serial_buffer, position
        )

    def move_absolute(self):
        check_error_code(lib.CC_MoveAbsolute, self._serial_buffer)

    def request_move_relative_distace(self):
        check_error_code(
            lib.CC_RequestMoveRelativeDistance, self._serial_buffer
        )

    def get_move_relative_distance(self) -> int:
        return lib.CC_GetMoveRelativeDistance(self._serial_buffer)

    def set_move_relative_distance(self, distance: int):
        check_error_code(
            lib.CC_SetMoveRelativeDistance, self._serial_buffer, distance
        )

    def move_relative_distance(self):
        check_error_code(lib.CC_MoveRelativeDistance, self._serial_buffer)

    def request_dc_pid_params(self):
        check_error_code(lib.CC_RequestDCPIDParams, self._serial_buffer)

    def get_dc_pid_params(self) -> DCPIDParameters:
        params = ffi.new("MOT_DC_PIDParameters *")
        check_error_code(lib.CC_GetDCPIDParams, self._serial_buffer, params)
        return DCPIDParameters(
            params.proportionalGain, params.integralGain,
            params.differentialGain, params.integralLimit,
            params.parameterFilter
        )

    def set_dc_pid_params(self, params: DCPIDParameters):
        p = ffi.new("MOT_DC_PIDParameters *")
        p.proportionalGain = params.proportional_gain
        p.integralGain = params.integral_gain
        p.differentialGain = params.differential_gain
        p.integralLimit = params.integral_limit
        p.parameterFilter = params.parameter_filter
        check_error_code(lib.CC_SetDCPIDParams, self._serial_buffer, p)

    def suspend_move_messages(self):
        check_error_code(lib.CC_SuspendMoveMessages, self._serial_buffer)

    def resume_move_messages(self):
        check_error_code(lib.CC_ResumeMoveMessages, self._serial_buffer)

    def request_status_bits(self):
        check_error_code(lib.CC_RequestStatusBits, self._serial_buffer)

    def get_status_bits(self) -> "KCubeDCServo.StatusBits":
        status_bits = lib.CC_GetStatusBits(self._serial_buffer)
        bits = self._read_status_bits(status_bits)
        return KCubeDCServo.StatusBits(
            at_cw_hardware_limit_switch=bits[0],
            at_ccw_hardware_limit_switch=bits[1],
            is_motor_moving_cw=bits[4],
            is_motor_moving_ccw=bits[5],
            is_motor_jogging_cw=bits[6],
            is_motor_jogging_ccw=bits[7],
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

    def request_settings(self):
        check_error_code(lib.CC_RequestSettings, self._serial_buffer)

    def get_stage_axis_limits(self) -> TravelLimits:
        return TravelLimits(
            lib.CC_GetStageAxisMinPos(self._serial_buffer),
            lib.CC_GetStageAxisMaxPos(self._serial_buffer)
        )

    def set_stage_axis_limits(self, stage_axis_limits: TravelLimits):
        check_error_code(
            lib.CC_SetStageAxisLimits, self._serial_buffer,
            stage_axis_limits.min_position, stage_axis_limits.max_position
        )

    def get_motor_travel_mode(self) -> TravelModes:
        return TravelModes(lib.CC_GetMotorTravelMode(self._serial_buffer))

    def set_motor_travel_mode(self, mode: TravelModes):
        mode = ffi.cast("MOT_TravelModes", mode)
        check_error_code(lib.CC_SetMotorTravelMode, self._serial_buffer, mode)

    def get_motor_params(self) -> MotorParameters:
        steps_per_rev = ffi.new("double *")
        gear_box_ratio = ffi.new("double *")
        pitch = ffi.new("double *")
        check_error_code(
            lib.CC_GetMotorParamsExt, self._serial_buffer, steps_per_rev,
            gear_box_ratio, pitch
        )
        return MotorParameters(steps_per_rev[0], gear_box_ratio[0], pitch[0])

    def set_motor_params(self, params: MotorParameters):
        check_error_code(
            lib.CC_GetMotorParamsExt, self._serial_buffer, params.steps_per_rev,
            params.gear_box_ratio, params.pitch
        )

    def get_motor_velocity_limits(self) -> MotorVelocityLimits:
        max_vel = ffi.new("double *")
        max_acc = ffi.new("double *")
        check_error_code(
            lib.CC_GetMotorVelocityLimits, self._serial_buffer, max_vel, max_acc
        )
        return MotorVelocityLimits(max_vel[0], max_acc[0])

    def set_motor_velocity_limits(self, limits: MotorVelocityLimits):
        check_error_code(
            lib.CC_SetMotorVelocityLimits, self._serial_buffer,
            limits.max_velocity, limits.max_acceleration
        )

    def reset_rotation_modes(self):
        check_error_code(lib.CC_ResetRotationModes, self._serial_buffer)

    def set_rotation_modes(
        self, mode: MovementModes, direction: MovementDirections
    ):
        mode = ffi.cast("MOT_MovementModes", mode)
        direction = ffi.cast("MOT_MovementDirections", direction)
        check_error_code(
            lib.CC_SetRotationModes, self._serial_buffer, mode, direction
        )

    def get_motor_travel_limits(self) -> TravelLimits:
        min_pos = ffi.new("double *")
        max_pos = ffi.new("double *")
        check_error_code(
            lib.CC_GetMotorTravelLimits, self._serial_buffer, min_pos, max_pos
        )
        return TravelLimits(min_pos[0], max_pos[0])

    def set_motor_travel_limits(self, limits: TravelLimits):
        check_error_code(
            lib.CC_SetMotorTravelLimits, self._serial_buffer,
            limits.min_position, limits.max_position
        )

    def request_digital_outputs(self):
        check_error_code(lib.CC_RequestDigitalOutputs, self._serial_buffer)

    def get_digital_outputs(self) -> int:
        return lib.CC_GetDigitalOutputs(self._serial_buffer)

    def set_digital_outputs(self, output: int):
        check_error_code(lib.CC_SetDigitalOutputs, self._serial_buffer, output)

    def real_value_from_device_unit(
        self, device_unit: int, unit_type: UnitType
    ):
        real_value = ffi.new("double *")
        check_error_code(
            lib.CC_GetRealValueFromDeviceUnit, self._serial_buffer, device_unit,
            real_value, unit_type
        )
        return real_value[0]

    def device_unit_from_real_value(
        self, real_value: float, unit_type: UnitType
    ):
        device_unit = ffi.new("int *")
        check_error_code(
            lib.CC_GetDeviceUnitFromRealValue, self._serial_buffer, real_value,
            device_unit, unit_type
        )
        return device_unit[0]

    def clear_msg_queue(self):
        lib.CC_ClearMessageQueue(self._serial_buffer)

    def get_msg_queue_size(self) -> int:
        return lib.CC_MessageQueueSize(self._serial_buffer)

    def get_next_msg(self):
        msg_type = ffi.new("WORD *")
        msg_id = ffi.new("WORD *")
        msg_data = ffi.new("DWORD *")
        success = lib.CC_GetNextMessage(
            self._serial_buffer, msg_type, msg_id, msg_data
        )
        return success, msg_type[0], msg_id[0], msg_data[0]

    def wait_for_msg(self):
        msg_type = ffi.new("WORD *")
        msg_id = ffi.new("WORD *")
        msg_data = ffi.new("DWORD *")
        success = lib.CC_WaitForMessage(
            self._serial_buffer, msg_type, msg_id, msg_data
        )
        return success, msg_type[0], msg_id[0], msg_data[0]

    led_switches = property(get_led_switches, set_led_switches)
    software_version = property(get_software_version)
    is_connected = property(check_connection)
    polling_duration = property(get_polling_duration)
    hardware_info = property(get_hardware_info)
    msg_queue_size = property(get_msg_queue_size)
    can_device_lock_front_panel = property(get_can_device_lock_front_panel)
    is_front_panel_locked = property(
        get_front_panel_locked, set_front_panel_locked
    )
    hub_bay = property(get_hub_bay)
    number_positions = property(get_number_positions)
    position = property(get_position)
    can_home = property(get_can_home)
    can_move_without_homing_first = property(get_can_move_without_homing_first)
    homing_params = property(get_homing_params, set_homing_params)
    jog_params = property(get_jog_params, set_jog_params)
    velocity_params = property(get_velocity_params, set_velocity_params)
    backlash = property(get_backlash, set_backlash)
    position_counter = property(get_position_counter, set_position_counter)
    encoder_counter = property(get_encoder_counter, set_encoder_counter)
    limit_switch_params = property(
        get_limit_switch_params, set_limit_switch_params
    )
    software_limit_mode = property(
        get_software_limit_mode, set_softwarre_limit_mode
    )
    mmi_params = property(get_mmi_params, set_mmi_params)
    trigger_config_params = property(
        get_trigger_config_params, set_trigger_config_params
    )
    position_trigger_params = property(
        get_position_trigger_params, set_position_trigger_params
    )
    absolute_position = property(
        get_move_absolute_position, set_move_absolute_position
    )
    relative_distance = property(
        get_move_relative_distance, set_move_relative_distance
    )
    dc_pid_params = property(get_dc_pid_params, set_dc_pid_params)
    status_bits = property(get_status_bits)
    stage_axis_limits = property(get_stage_axis_limits, set_stage_axis_limits)
    motor_travel_mode = property(get_motor_travel_mode, set_motor_travel_mode)
    motor_params = property(get_motor_params, set_motor_params)
    motor_velocity_limits = property(
        get_motor_velocity_limits, set_motor_velocity_limits
    )
    motor_travel_limits = property(
        get_motor_travel_limits, set_motor_travel_limits
    )
    digital_outputs = property(get_digital_outputs, set_digital_outputs)
