import os
from pathlib import Path
from cffi import FFI

ffibuilder = FFI()
path = Path(__file__).parent

with open(path / "Solenoid.cdef", "r") as f:
    ffibuilder.cdef(f.read())

ffibuilder.set_source(
    "_thorlabs.motions_control.kcube.solenoid",
    """#include "Solenoid.h" """,
    libraries=["Thorlabs.MotionControl.KCube.Solenoid"],
    include_dirs=[path],
    library_dirs=[os.getenv("THORLABS_KINESIS_PATH", "./lib")]
)

if __name__ == "__main__":
    ffibuilder.compile(verbose=True)