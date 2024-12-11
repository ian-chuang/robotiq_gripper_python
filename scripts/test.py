from robotiq_gripper_python import RobotiqGripper
import time

gripper = RobotiqGripper(comport="/dev/tty.usbserial-DAT27MH")

gripper.activate_gripper()

# close slowly
gripper.goto(pos=255, vel=0, force=0, block=True)

# open fast
gripper.goto(pos=0, vel=255, force=255, block=True)

# non-blocking close
for i in range(255):
    gripper.goto(pos=i, vel=255, force=255, block=False)
    time.sleep(0.01)

gripper.shutdown()