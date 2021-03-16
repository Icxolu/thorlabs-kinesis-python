import os
from pathlib import Path
from cffi import FFI

ffibuilder = FFI()
path = Path(__file__).parent

with open(path / "FilterFlipper.cdef", "r") as f:
    ffibuilder.cdef(f.read())

ffibuilder.set_source(
    "_thorlabs.motions_control.filter_flipper",
    """#include "FilterFlipper.h" """,
    libraries=["Thorlabs.MotionControl.FilterFlipper"],
    include_dirs=[path],
    library_dirs=[os.getenv("THORLABS_KINESIS_PATH", "./lib")]
)

if __name__ == "__main__":
    ffibuilder.compile(verbose=True)