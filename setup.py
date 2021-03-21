from setuptools import setup, find_namespace_packages

setup(
    name="Thorlabs-Kinesis-Python",
    packages=find_namespace_packages(),
    setup_requires=["cffi>=1.0.0"],
    cffi_modules=[
        "MotionControl/IntegratedStepperMotors/IntegratedStepperMotors_build.py:ffibuilder",
        "MotionControl/FilterFlipper/FilterFlipper_build.py:ffibuilder",
        "MotionControl/KCube/StepperMotor/StepperMotor_build.py:ffibuilder",
        "MotionControl/KCube/DCServo/DCServo_build.py:ffibuilder",
        "MotionControl/KCube/Solenoid/Solenoid_build.py:ffibuilder",
    ],
    install_requires=["cffi>=1.0.0"]
)