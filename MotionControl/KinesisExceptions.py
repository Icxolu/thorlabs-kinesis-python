from _thorlabs.motions_control.integrated_stepper_motors import lib


class ThorlabsKinesisException(Exception):
    pass


class InvalidHandleException(ThorlabsKinesisException):
    pass


class DeviceNotFoundException(ThorlabsKinesisException):
    pass


class DeviceNotOpenedException(ThorlabsKinesisException):
    pass


class IOException(ThorlabsKinesisException):
    pass


class InsufficientResourcesException(ThorlabsKinesisException):
    pass


class InvalidParameterException(ThorlabsKinesisException):
    pass


class DeviceNotPresentException(ThorlabsKinesisException):
    pass


class IncorrectDeviceException(ThorlabsKinesisException):
    pass


def check_error_code(func):

    def _inner(*args, **kwargs):
        error_code = func(*args, **kwargs)
        if error_code == lib.FT_OK:
            return
        elif error_code == lib.FT_InvalidHandle:
            raise InvalidHandleException()
        elif error_code == lib.FT_DeviceNotFound:
            raise DeviceNotFoundException()
        elif error_code == lib.FT_DeviceNotOpened:
            raise DeviceNotOpenedException()
        elif error_code == lib.FT_IOError:
            raise IOException()
        elif error_code == lib.FT_InsufficientResources:
            raise InsufficientResourcesException()
        elif error_code == lib.FT_InvalidParameter:
            raise InvalidParameterException()
        elif error_code == lib.FT_DeviceNotPresent:
            raise DeviceNotPresentException()
        elif error_code == lib.FT_IncorrectDevice:
            raise IncorrectDeviceException()

    return _inner