# robotiq_gripper_python
Simple easy to use python driver for Robotiq 2F-85 and 2F-140 via USB using modbus RTU adapted from https://github.com/PickNikRobotics/robotiq_85_gripper

I noticed it was difficult to find a properly working standalone python driver. Many other repos don't work anymore, require dependencies without proper versions, or are very slow. This driver is very responsive and the only dependency is pyserial.

## Installation

Install directly from the repository:

```bash
pip install git+https://github.com/ian-chuang/robotiq_gripper_python.git
```

## Usage

Here's an example of how to use the driver:

```python
from robotiq_gripper_python import RobotiqGripper
import time

# Initialize the gripper (replace with your COM port)
gripper = RobotiqGripper(comport="/dev/tty.usbserial-DAT27MH")

# Activate the gripper
gripper.activate_gripper()

# Close the gripper slowly
gripper.goto(pos=255, vel=0, force=0, block=True)

# Open the gripper quickly
gripper.goto(pos=0, vel=255, force=255, block=True)

# Perform a non-blocking close with incremental positions
for i in range(255):
    gripper.goto(pos=i, vel=255, force=255, block=False)
    time.sleep(0.01)

# Shutdown the gripper
gripper.shutdown()
```