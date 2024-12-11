from setuptools import setup, find_packages

setup(
    name="robotiq_gripper_python",  # Package name
    version="0.1.0",    # Version
    description="A sample Python package",
    packages=find_packages(),  # Automatically find packages
    install_requires=[
        # numpy
        "pyserial",
    ],
)
