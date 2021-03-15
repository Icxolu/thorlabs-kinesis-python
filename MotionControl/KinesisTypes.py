from enum import unique, IntEnum
from typing import NamedTuple


@unique
class MessageType(IntEnum):
    GenericDevice = 0
    GenericPiezo = 1
    GenericMotor = 2
    GenericDCMotor = 3
    GenericSimpleMotor = 4
    RackDevice = 5
    Laser = 6
    TECCtlr = 7
    Quad = 8
    NanoTrak = 9
    Specialized = 10
    Solenoid = 11


class MessageID(IntEnum):
    pass


@unique
class GenericDeviceMID(MessageID):
    SettingsInitialized = 0
    SettingsUpdated = 1
    SettingsExtern = 2
    Error = 3
    Close = 4
    SettingsReset = 5


@unique
class GenericMotorsMID(MessageID):
    Homed = 0
    Moved = 1
    Stopped = 2
    LimitUpdated = 3


@unique
class GenecricDCMotorsMID(MessageID):
    Error = 0
    Status = 1


@unique
class GenericPiezoMID(MessageID):
    MaxVoltageChanged = 0
    ControlModeChanged = 1
    StatusChanged = 2
    MaxTravelChanged = 3
    TSG_Status = 4
    TSG_DisplayModeChanged = 5


@unique
class RackDevicesMID(MessageID):
    RackCountEstablished = 0
    RackBayState = 1


@unique
class QuadMID(MessageID):
    statusChanged = 0


@unique
class TEC_MID(MessageID):
    StatusChanged = 0
    DisplaySettingsChanged = 2
    FeedbackParamsChanged = 3


@unique
class LaserMID(MessageID):
    StatusChanged = 0
    ControlSourceChanged = 1
    DisplayModeChanged = 2


@unique
class SolenoidMID(MessageID):
    StatusChanged = 0


@unique
class NanoTrakMID(MessageID):
    StatusChanged = 0


@unique
class JogModes(IntEnum):
    JogModeUndefined = 0x00
    Continuous = 0x01
    SingleStep = 0x02


@unique
class StopModes(IntEnum):
    StopModeUndefined = 0x00
    Immediate = 0x01
    Profiled = 0x02


@unique
class MotorTypes(IntEnum):
    NotMotor = 0
    DCMotor = 1
    StepperMotor = 2
    BrushlessMotor = 3
    CustomMotor = 100


@unique
class TravelDirection(IntEnum):
    TravelDirectionUndefined = 0x00
    Forwards = 0x01
    Reverse = 0x02


@unique
class HomeLimitSwitchDirection(IntEnum):
    LimitSwitchDirectionUndefined = 0x00
    ReverseLimitSwitch = 0x01
    ForwardLimitSwitch = 0x04


@unique
class ButtonModes(IntEnum):
    ButtonModeUndefined = 0x00
    JogMode = 0x01
    Preset = 0x02


@unique
class LimitSwitchModes(IntEnum):
    LimitSwitchModeUndefined = 0x00
    LimitSwitchIgnoreSwitch = 0x01
    LimitSwitchMakeOnContact = 0x02
    LimitSwitchBreakOnContact = 0x03
    LimitSwitchMakeOnHome = 0x04
    LimitSwitchBreakOnHome = 0x05
    PMD_Reserved = 0x06
    LimitSwitchIgnoreSwitchSwapped = 0x81
    LimitSwitchMakeOnContactSwapped = 0x82
    LimitSwitchBreakOnContactSwapped = 0x83
    LimitSwitchMakeOnHomeSwapped = 0x84
    LimitSwitchBreakOnHomeSwapped = 0x85


@unique
class LimitSwitchSWModes(IntEnum):
    LimitSwitchSWModeUndefined = 0x00
    LimitSwitchIgnored = 0x01
    LimitSwitchStopImmediate = 0x02
    LimitSwitchStopProfiled = 0x03
    LimitSwitchIgnored_Rotational = 0x81
    LimitSwitchStopImmediate_Rotational = 0x82
    LimitSwitchStopProfiled_Rotational = 0x83


@unique
class LimitsSoftwareApproachPolicy(IntEnum):
    DisallowIllegalMoves = 0
    AllowPartialMoves = 1
    AllowAllMoves = 2


@unique
class UnitType(IntEnum):
    Distance = 0
    Velocity = 1
    Acceleration = 2


@unique
class TravelModes(IntEnum):
    TravelModeUndefined = 0
    Linear = 0x01
    Rotational = 0x02


@unique
class MovementModes(IntEnum):
    LinearRange = 0x00
    RotationalUnlimited = 0x01
    RotationalWrapping = 0x02


@unique
class MovementDirections(IntEnum):
    Quickest = 0x00
    Forwards = 0x01
    Reverse = 0x02


@unique
class WheelMode(IntEnum):
    Velocity = 0x01
    Jog = 0x02
    MoveAbsolute = 0x03


@unique
class WheelDirectionSense(IntEnum):
    Positive = 0x01
    Negative = 0x02


@unique
class DirectionSense(IntEnum):
    Normal = 0x00
    Reverse = 0x01


@unique
class TriggerPortMode(IntEnum):
    TrigDisabled = 0x00
    TrigIn_GPI = 0x01
    TrigIn_RelativeMove = 0x02
    TrigIn_AbsoluteMove = 0x03
    TrigIn_Home = 0x04
    TrigIn_Stop = 0x05
    TrigOut_GPO = 0x0A
    TrigOut_InMotion = 0x0B
    TrigOut_AtMaxVelocity = 0x0C
    TrigOut_AtPositionSteps = 0x0D
    TrigOut_Synch = 0x0E


@unique
class TriggerPortPolarity(IntEnum):
    TrigPolarityHigh = 0x01
    TrigPolarityLow = 0x02


@unique
class PIDLoopMode(IntEnum):
    LoopModeDisabled = 0x00
    OpenLoopMode = 0x01
    ClosedLoopMode = 0x02


class HardwareInfo(NamedTuple):
    serial_number: int
    model_number: str
    type: int
    firmware_version: str
    notes: str
    device_dependent_data: bytes
    hardware_version: int
    modification_state: int
    num_channels: int


class VelocityParameters(NamedTuple):
    acceleration: int
    max_velocity: int
    min_velocity: int = 0


class JogParameters(NamedTuple):
    mode: JogModes
    step_size: int
    vel_params: VelocityParameters
    stop_mode: StopModes


class HomingParameters(NamedTuple):
    direction: TravelDirection
    limit_switch: HomeLimitSwitchDirection
    velocity: int
    offset_distance: int


class PowerParameters(NamedTuple):
    rest_percentage: int
    move_percentage: int


class ButtonParameters(NamedTuple):
    button_mode: ButtonModes
    left_button_position: int
    right_button_position: int
    timout: int = 0


class PotentiometerStep(NamedTuple):
    threshold_deflection: int
    velocity: int


class PotentiometerSteps(NamedTuple):
    potentiometer_step_parameter_1: PotentiometerStep
    potentiometer_step_parameter_2: PotentiometerStep
    potentiometer_step_parameter_3: PotentiometerStep
    potentiometer_step_parameter_4: PotentiometerStep


class PotentiometerParameters(NamedTuple):
    index: int
    threshold_deflection: int
    velocity: int


class LimitSwitchParameters(NamedTuple):
    clockwise_hardware_limit: LimitSwitchModes
    anticlockwise_hardware_limit: LimitSwitchModes
    clockwise_position: int
    anticlockwise_position: int
    soft_limit_mode: LimitSwitchSWModes


class TravelLimits(NamedTuple):
    min_position: int
    max_position: int


class MotorParameters(NamedTuple):
    steps_per_rev: float
    gear_box_ratio: float
    pitch: float


class MotorVelocityLimits(NamedTuple):
    max_velocity: float
    max_acceleration: float


class MMIParameters(NamedTuple):
    wheel_mode: WheelMode
    wheel_max_velocity: int
    wheel_acceleration: int
    wheel_direction_sense: DirectionSense
    preset_pos_1: int
    preset_pos_2: int
    display_intensity: int
    display_timeout: int
    display_dim_intensity: int


class TriggerConfigParameters(NamedTuple):
    trigger_1_mode: TriggerPortMode
    trigger_1_polarity: TriggerPortPolarity
    trigger_2_mode: TriggerPortMode
    trigger_2_polarity: TriggerPortPolarity


class PositionTriggerParameters(NamedTuple):
    trigger_start_position_fwd: int
    trigger_interval_fwd: int
    trigger_pulse_count_fwd: int
    trigger_start_position_rev: int
    trigger_interval_rev: int
    trigger_pulse_count_rev: int
    trigger_pulse_width: int
    cycle_count: int


class PIDLoopEncoderParameters(NamedTuple):
    loop_mode: PIDLoopMode
    proportional_gain: int
    integral_gain: int
    differential_gain: int
    pid_output_limit: int
    pid_tolerance: int

class DCPIDParameters(NamedTuple):
    proportional_gain: int
    integral_gain: int
    differential_gain: int
    integral_limit: int
    parameter_filter: int