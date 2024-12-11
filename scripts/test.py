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