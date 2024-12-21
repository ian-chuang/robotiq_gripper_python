from robotiq_gripper_python import RobotiqGripper
import time

if __name__ == "__main__":
    gripper = RobotiqGripper(comport="/dev/tty.usbserial-DAT27MH")

    gripper.start()

    for i in range(3):
        # close slowly
        gripper.move(pos=255, vel=30, force=30, block=True)
        # open slowly
        gripper.move(pos=0, vel=30, force=30, block=True)

    for i in range(255):
        gripper.move(pos=i, vel=255, force=255, block=False)
        time.sleep(0.01)

    gripper.shutdown()