import os
from pathlib import Path
from cffi import FFI

ffibuilder = FFI()
path = Path(__file__).parent

with open(path / "IntegratedStepperMotors.cdef", "r") as f:
    ffibuilder.cdef(f.read())

ffibuilder.set_source(
    "_thorlabs.motions_control.integrated_stepper_motors",
    """#include "IntegratedStepperMotors.h" """,
    libraries=["Thorlabs.MotionControl.IntegratedStepperMotors"],
    include_dirs=[path],
    library_dirs=[os.getenv("THORLABS_KINESIS_PATH", "./lib")]
)

if __name__ == "__main__":
    ffibuilder.compile(verbose=True)