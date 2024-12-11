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

gripper = RobotiqGripper(comport="/dev/tty.usbserial-DAT27MH")
gripper.deactivate_gripper()
time.sleep(0.5)
gripper.activate_gripper()
time.sleep(0.5)

# open slowly
gripper.goto(pos=0, vel=30, force=30)
time.sleep(3)

# close slowly
gripper.goto(pos=255, vel=30, force=30)
time.sleep(2)

# open slowly
gripper.goto(pos=0, vel=30, force=30)
time.sleep(3)

for i in range(255):
    gripper.goto(pos=i, vel=255, force=255)
    time.sleep(0.01)

gripper.shutdown()
```