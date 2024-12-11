import serial
from robotiq_gripper_python.gripper_io import GripperIO
from robotiq_gripper_python.modbus_crc import verify_modbus_rtu_crc
import threading
import time

class RobotiqGripper:
    def __init__(self, num_grippers=1, comport='/dev/ttyUSB0', baud=115200, control_hz=60):
        try:
            self.ser = serial.Serial(comport, baud, timeout=0.2)
        except:
            self.init_success = False
            return

        self._gripper = []
        self._num_grippers = num_grippers
        self._locks = [threading.Lock() for _ in range(self._num_grippers)]

        for i in range(self._num_grippers):
            self._gripper.append(GripperIO(i))
        self.init_success = True
        self._shutdown_driver = False

        self._control_hz = control_hz

        self._threads = []
        self._thread_running = True
        for i in range(self._num_grippers):
            t = threading.Thread(target=self._gripper_loop, args=(i,), daemon=True)
            self._threads.append(t)
            t.start()

    def __del__(self):
        self.shutdown()

    def _gripper_loop(self, dev):
        while self._thread_running and not self._shutdown_driver:
            self._process_stat_cmd(dev)
            self._process_act_cmd(dev)
            time.sleep(1.0 / self._control_hz)

    def shutdown(self):
        self._thread_running = False
        for t in self._threads:
            t.join()

        self._shutdown_driver = True
        self.ser.close()

    def _process_act_cmd(self, dev=0):
        if dev >= self._num_grippers or self._shutdown_driver:
            return False
        try:
            # Step 1: Copy the bytes inside the lock
            with self._locks[dev]:
                act_cmd_bytes = self._gripper[dev].act_cmd_bytes

            # Step 2: Perform serial communication outside the lock
            self.ser.write(act_cmd_bytes)
            rsp = self.ser.read(8)
            rsp = [int(x) for x in rsp]
            if len(rsp) != 8:
                return False
            return verify_modbus_rtu_crc(rsp)
        except:
            return False

    def _process_stat_cmd(self, dev=0):
        try:
            # Step 1: Copy the bytes inside the lock
            with self._locks[dev]:
                stat_cmd_bytes = self._gripper[dev].stat_cmd_bytes

            # Step 2: Perform serial communication outside the lock
            self.ser.write(stat_cmd_bytes)
            rsp = self.ser.read(21)
            rsp = [int(x) for x in rsp]
            if len(rsp) != 21:
                return False
            return self._gripper[dev].parse_rsp(rsp)
        except:
            return False


    def activate_gripper(self, dev=0):
        if dev >= self._num_grippers:
            return
        with self._locks[dev]:
            self._gripper[dev].activate_gripper()

    def deactivate_gripper(self, dev=0):
        if dev >= self._num_grippers:
            return
        with self._locks[dev]:
            self._gripper[dev].deactivate_gripper()

    def activate_emergency_release(self, dev=0, open_gripper=True):
        if dev >= self._num_grippers:
            return
        with self._locks[dev]:
            self._gripper[dev].activate_emergency_release(open_gripper)

    def deactivate_emergency_release(self, dev=0):
        if dev >= self._num_grippers:
            return
        with self._locks[dev]:
            self._gripper[dev].deactivate_emergency_release()

    def goto(self, dev=0, pos=0.0, vel=1.0, force=1.0):
        if dev >= self._num_grippers:
            return
        with self._locks[dev]:
            self._gripper[dev].goto(pos, vel, force)

    def stop(self, dev=0):
        if dev >= self._num_grippers:
            return
        with self._locks[dev]:
            self._gripper[dev].stop()

    def is_ready(self, dev=0):
        if dev >= self._num_grippers:
            return False
        with self._locks[dev]:
            return self._gripper[dev].is_ready()

    def is_reset(self, dev=0):
        if dev >= self._num_grippers:
            return False
        with self._locks[dev]:
            return self._gripper[dev].is_reset()

    def is_moving(self, dev=0):
        if dev >= self._num_grippers:
            return False
        with self._locks[dev]:
            return self._gripper[dev].is_moving()

    def is_stopped(self, dev=0):
        if dev >= self._num_grippers:
            return False
        with self._locks[dev]:
            return self._gripper[dev].is_moving()

    def object_detected(self, dev=0):
        if dev >= self._num_grippers:
            return False
        with self._locks[dev]:
            return self._gripper[dev].object_detected()

    def get_fault_status(self, dev=0):
        if dev >= self._num_grippers:
            return 0
        with self._locks[dev]:
            return self._gripper[dev].get_fault_status()

    def get_pos(self, dev=0):
        if dev >= self._num_grippers:
            return 0
        with self._locks[dev]:
            return self._gripper[dev].get_pos()

    def get_req_pos(self, dev=0):
        if dev >= self._num_grippers:
            return 0
        with self._locks[dev]:
            return self._gripper[dev].get_req_pos()

    def get_current(self, dev=0):
        if dev >= self._num_grippers:
            return 0
        with self._locks[dev]:
            return self._gripper[dev].get_current()
